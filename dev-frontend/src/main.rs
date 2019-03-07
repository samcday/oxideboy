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

    println!("Loaded ROM {:?}", gb.hw.cart.rom_title());

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
                Event::KeyDown {
                    keycode: Some(Keycode::Up),
                    ..
                } => {
                    gb.hw.joypad.up = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Down),
                    ..
                } => {
                    gb.hw.joypad.down = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Right),
                    ..
                } => {
                    gb.hw.joypad.right = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Left),
                    ..
                } => {
                    gb.hw.joypad.left = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::A),
                    ..
                } => {
                    gb.hw.joypad.a = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::S),
                    ..
                } => {
                    gb.hw.joypad.b = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::Return),
                    ..
                } => {
                    gb.hw.joypad.start = true;
                }
                Event::KeyDown {
                    keycode: Some(Keycode::RShift),
                    ..
                } => {
                    gb.hw.joypad.select = true;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Up),
                    ..
                } => {
                    gb.hw.joypad.up = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Down),
                    ..
                } => {
                    gb.hw.joypad.down = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Right),
                    ..
                } => {
                    gb.hw.joypad.right = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Left),
                    ..
                } => {
                    gb.hw.joypad.left = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::A),
                    ..
                } => {
                    gb.hw.joypad.a = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::S),
                    ..
                } => {
                    gb.hw.joypad.b = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::Return),
                    ..
                } => {
                    gb.hw.joypad.start = false;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::RShift),
                    ..
                } => {
                    gb.hw.joypad.select = false;
                }

                Event::KeyUp {
                    keycode: Some(Keycode::L),
                    ..
                } => {
                    limit = !limit;
                    audio_device.clear();
                }
                Event::KeyUp {
                    keycode: Some(Keycode::P),
                    ..
                } => {
                    paused = !paused;
                }
                Event::KeyUp {
                    keycode: Some(Keycode::M),
                    ..
                } => {
                    if audio_device.status() == AudioStatus::Playing {
                        audio_device.clear();
                        audio_device.pause();
                    } else {
                        audio_device.resume();
                    }
                }

                // Event::KeyUp {keycode: Some(Keycode::LeftBracket), ..} => {
                //     save_state = Some(gameboy.save_state());
                // }
                // Event::KeyUp {keycode: Some(Keycode::RightBracket), ..} => {
                //     if let Some(ref data) = save_state {
                //         gameboy.load_state(&data[..]);
                //         gameboy.state.joypad = Default::default();
                //         gb_buffer.copy_from_slice(gameboy.state.ppu.framebuffer());
                //         audio_device.clear();
                //         audio_device.queue(&gameboy.state.apu.sample_queue[..]);
                //     }
                // }
                _ => {}
            }
        }

        if !paused {
            gb.hw.apu.sample_queue.clear();
            for _ in 0..17556 {
                gb.run_instruction();

                if gb.hw.new_frame {
                    gb_buffer.copy_from_slice(&gb.hw.ppu.framebuffer);
                }
            }

            let mut samples = &gb.hw.apu.sample_queue[..];
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
