use oxideboy::rewind::*;
use oxideboy::rom::Rom;
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
    let model = match model_str.as_str() {
        "DMG" => Model::DMGABC,
        "MGB" => Model::MGB,
        _ => Model::DMG0,
    };
    let run_bootrom = env::var("RUN_BOOTROM").ok().unwrap_or_else(|| String::from("0")) == "1";

    let rom = Rom::new(rom.into()).expect("Loading ROM failed");
    let mut gb = Gameboy::new(model, &rom, run_bootrom);
    let mut gb_context = Context::new(rom);
    let mut rewind_manager = RewindManager::new(&gb, MemoryStorageAdapter::new()).unwrap();

    println!("Loaded ROM {:?}", gb_context.rom.title);

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

    if env::var("MUTE").ok().unwrap_or_else(|| String::from("0")) != "1" {
        audio_device.resume();
    }

    let mut state: Vec<u8> = Vec::new();
    let mut limit = env::var("LIMIT").ok().unwrap_or_else(|| String::from("0")) == "1";
    let mut paused = false;
    let mut rewinding = false;
    let print_fps = env::var("PRINT_FPS").ok().unwrap_or_else(|| String::from("0")) == "1";

    let start = Instant::now();
    let mut next_frame = FRAME_TIME;

    let mut update_fb = |framebuffer: &[u16]| {
        texture
            .with_lock(None, |buffer: &mut [u8], _: usize| {
                let framebuffer = unsafe { slice::from_raw_parts(framebuffer.as_ptr() as *mut u8, 160 * 144 * 2) };
                buffer.copy_from_slice(&framebuffer);
            })
            .unwrap();

        canvas.copy(&texture, None, Some(Rect::new(0, 0, 320, 288))).unwrap();
        canvas.present();
    };

    let queue_samples = |gb_context: &mut Context| {
        gb_context.drain_audio_samples(|samples| {
            if audio_device.status() == AudioStatus::Playing {
                audio_device.queue(samples);
            }
        });
    };

    let mut frame_count = 0;
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
                    Keycode::LCtrl => {
                        // When we stop rewinding we need to reset all joypad buttons to unpressed.
                        // Otherwise, whatever was pressed at the time of the frame we rewinded (rewound?) to will still
                        // be pressed.
                        gb.joypad.up = false;
                        gb.joypad.down = false;
                        gb.joypad.right = false;
                        gb.joypad.left = false;
                        gb.joypad.a = false;
                        gb.joypad.b = false;
                        gb.joypad.start = false;
                        gb.joypad.select = false;
                        rewinding = false;
                    }
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
                    Keycode::LCtrl => rewinding = true,

                    Keycode::F7 => {
                        state.clear();
                        save_state(&gb, &gb_context, &mut state).unwrap();
                    }
                    Keycode::F8 => {
                        if !state.is_empty() {
                            load_state(&mut gb, &mut gb_context, &state[..]).unwrap();
                            audio_device.clear();
                            queue_samples(&mut gb_context);
                            update_fb(&gb_context.current_framebuffer);
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
            if rewinding {
                rewind_manager.rewind_frame(&mut gb, &mut gb_context);
                frame_count = gb.frame_count;
            } else {
                for _ in 0..17556 {
                    gb.run_instruction(&mut gb_context);

                    if gb.frame_count > frame_count {
                        frame_count = gb.frame_count;
                        rewind_manager.notify_frame(&mut gb);
                    }
                }
            }

            // if throwaway_samples {
            //     let mut samples = samples.skip_while(|_| {

            //     });
            //     if throwaway_count > samples.len() {
            //         throwaway_count -= samples.len();
            //         samples = &[];
            //     } else {
            //         samples = &samples[throwaway_count..];
            //         throwaway_samples = false;
            //     }
            // }

            queue_samples(&mut gb_context);
        }

        if limit {
            update_fb(&gb_context.current_framebuffer);
            let elapsed = start.elapsed();
            if elapsed < next_frame {
                std::thread::sleep(next_frame - elapsed);
            }
            next_frame += FRAME_TIME;
        } else if start.elapsed() >= next_frame {
            update_fb(&gb_context.current_framebuffer);
            next_frame += FRAME_TIME;
        }

        frames += 1;
    }

    Ok(())
}
