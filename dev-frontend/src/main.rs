use oxideboy::joypad::*;
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
    let mut frame_count = 0;

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

    fn map_keycode_to_joypad(keycode: Option<sdl2::keyboard::Keycode>) -> Option<Button> {
        match keycode {
            Some(Keycode::Up) => Some(Button::Up),
            Some(Keycode::Down) => Some(Button::Down),
            Some(Keycode::Right) => Some(Button::Right),
            Some(Keycode::Left) => Some(Button::Left),
            Some(Keycode::A) => Some(Button::A),
            Some(Keycode::S) => Some(Button::B),
            Some(Keycode::Return) => Some(Button::Start),
            Some(Keycode::RShift) => Some(Button::Select),
            _ => None,
        }
    }

    let mut frames = 0.0;
    let sec = Duration::from_secs(1);
    let mut fps_timer = Instant::now();

    'running: loop {
        frames += 1.0;
        if fps_timer.elapsed() >= sec {
            if print_fps {
                println!("FPS: {}", frames / (fps_timer.elapsed().as_millis() as f32 / 1000.0));
            }
            frames = 0.0;
            fps_timer = Instant::now();
        }

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit { .. }
                | Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => break 'running,

                Event::KeyUp { keycode, .. } | Event::KeyDown { keycode, .. }
                    if map_keycode_to_joypad(keycode).is_some() =>
                {
                    let pressed = if let Event::KeyDown { .. } = event { true } else { false };
                    gb.joypad
                        .state
                        .set_button(map_keycode_to_joypad(keycode).unwrap(), pressed);
                    rewind_manager.notify_input(&gb);
                }

                Event::KeyUp {
                    keycode: Some(Keycode::LCtrl),
                    ..
                } => {
                    // When we stop rewinding we need to reset all joypad buttons to unpressed.
                    // Otherwise, whatever was pressed at the time of the frame we rewinded (rewound?) to will still
                    // be pressed.
                    gb.joypad.state.clear();
                    rewinding = false;
                }

                Event::KeyDown {
                    keycode: Some(keycode), ..
                } => match keycode {
                    Keycode::LCtrl => rewinding = true,
                    Keycode::P => paused = !paused,
                    Keycode::L => limit = !limit,
                    Keycode::M => gb_context.enable_audio = !gb_context.enable_audio,

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
    }

    Ok(())
}
