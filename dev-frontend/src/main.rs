use oxideboy::*;
use sdl2::audio::{AudioSpecDesired, AudioStatus};
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use std::env;
use std::fs::File;
use std::io::prelude::*;
use std::slice;
use std::time::{Duration, Instant};

const FRAME_TIME: Duration = Duration::from_nanos((1_000_000_000.0 / (1_048_576.0 / 17556.0)) as u64);

type Result<T> = std::result::Result<T, Box<std::error::Error>>;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let model_str = env::var("MODEL").ok().unwrap_or_else(|| String::from("DMG0"));
    let model = if model_str == "DMG" { Model::DMG } else { Model::DMG0 };

    let mut gb = Gameboy::new(model, rom);

    println!("Loaded ROM {:?}", gb.cart.rom_title());

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let audio_subsystem = sdl_context.audio().unwrap();
    let audio_device = audio_subsystem
        .open_queue(
            None,
            &AudioSpecDesired {
                freq: Some(44100),
                channels: Some(2),
                samples: None,
            },
        )
        .unwrap();

    let window = video_subsystem
        .window("oxideboy", 320, 288)
        .position_centered()
        .opengl()
        .build()
        .unwrap();
    let mut canvas = window.into_canvas().build().unwrap();
    let texture_creator = canvas.texture_creator();
    let mut texture = texture_creator
        .create_texture_streaming(PixelFormatEnum::RGB565, 160, 144)
        .unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();

    if env::var("RUN_BOOTROM").ok().unwrap_or_else(|| String::from("0")) != "1" {
        gb.skip_bootrom();
    }

    if env::var("MUTE").ok().unwrap_or_else(|| String::from("0")) != "1" {
        audio_device.resume();
    }

    let mut save_state: Vec<u8> = Vec::new();
    let mut gb_buffer = [0; ppu::SCREEN_SIZE];
    let mut limit = false;
    let mut paused = false;
    let print_fps = env::var("PRINT_FPS").ok().unwrap_or_else(|| String::from("0")) == "1";

    let start = Instant::now();
    let mut next_frame = FRAME_TIME;

    let mut update_fb = |gb_buffer: &[u16]| {
        texture
            .with_lock(None, |buffer: &mut [u8], _: usize| {
                // TODO: clippy doesn't like this because it can apparently lead to undefined behaviour.
                // Spent a few mins googling and couldn't figure out the idiomatic way to do this...
                #[allow(clippy::cast_ptr_alignment)]
                let buffer = unsafe { slice::from_raw_parts_mut(buffer.as_ptr() as *mut u16, 160 * 144) };
                buffer.copy_from_slice(&gb_buffer[..]);
            })
            .unwrap();

        canvas.clear();
        canvas.copy(&texture, None, Some(Rect::new(0, 0, 320, 288))).unwrap();
        canvas.present();
    };

    let mut throwaway_samples = true;
    let mut throwaway_count = 44100;
    let mut frames = 0;
    let mut fps_timer = Instant::now();
    let sec = Duration::from_secs(1);
    'running: loop {
        if fps_timer.elapsed() >= sec {
            if print_fps {
                println!("FPS: {}", frames);
            }
            fps_timer = Instant::now();
            frames = 0;
        }

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,

                Event::KeyUp {
                    keycode: Some(keycode), ..
                } => match keycode {
                    Keycode::Up => gb.joypad.up = false,
                    Keycode::Down => gb.joypad.down = false,
                    Keycode::Right => gb.joypad.right = false,
                    Keycode::Left => gb.joypad.left = false,
                    Keycode::A => gb.joypad.a = false,
                    Keycode::S => gb.joypad.b = false,
                    Keycode::Return => gb.joypad.start = false,
                    Keycode::RShift => gb.joypad.select = false,
                    _ => {}
                },

                Event::KeyDown {
                    keycode: Some(keycode), ..
                } => match keycode {
                    Keycode::Escape => break 'running,
                    Keycode::Up => gb.joypad.up = true,
                    Keycode::Down => gb.joypad.down = true,
                    Keycode::Right => gb.joypad.right = true,
                    Keycode::Left => gb.joypad.left = true,
                    Keycode::A => gb.joypad.a = true,
                    Keycode::S => gb.joypad.b = true,
                    Keycode::Return => gb.joypad.start = true,
                    Keycode::RShift => gb.joypad.select = true,

                    Keycode::F7 => {
                        save_state.clear();
                        gb.save_state(&mut save_state);
                    }
                    Keycode::F8 => {
                        if !save_state.is_empty() {
                            gb.load_state(&save_state);
                            audio_device.clear();
                        }
                    }
                    Keycode::L => {
                        limit = !limit;
                        audio_device.clear();
                    }
                    Keycode::P => {
                        paused = !paused;
                    }
                    Keycode::M => {
                        if audio_device.status() == AudioStatus::Playing {
                            audio_device.clear();
                            audio_device.pause();
                        } else {
                            audio_device.resume();
                        }
                    }
                    _ => {}
                },
                _ => {}
            }
        }

        if !paused {
            gb.apu.sample_queue.clear();
            for _ in 0..17556 {
                gb.run_instruction();

                if gb.new_frame {
                    gb_buffer.copy_from_slice(&gb.ppu.framebuffer);
                }
            }

            let mut samples = &gb.apu.sample_queue[..];
            if throwaway_samples {
                if throwaway_count > samples.len() {
                    throwaway_count -= samples.len();
                    samples = &[];
                } else {
                    samples = &samples[throwaway_count..];
                    throwaway_samples = false;
                }
            }

            if audio_device.status() == AudioStatus::Playing {
                if !limit {
                    audio_device.clear();
                }
                audio_device.queue(samples);
            }
        }

        if limit {
            update_fb(&gb_buffer);

            let elapsed = start.elapsed();
            if elapsed < next_frame {
                std::thread::sleep(next_frame - elapsed);
            }
            next_frame += FRAME_TIME;
        } else if start.elapsed() >= next_frame {
            update_fb(&gb_buffer);
            next_frame += FRAME_TIME;
        }

        frames += 1;
    }

    Ok(())
}
