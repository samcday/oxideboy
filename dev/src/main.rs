extern crate gameboy;
extern crate sdl2;

use std::io::prelude::*;
use std::fs::File;
use std::env;
use std::time::{Duration, Instant};
use std::slice;

use sdl2::audio::{AudioSpecDesired, AudioStatus};
use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

type Result<T> = std::result::Result<T, Box<std::error::Error>>;

const FRAME_TIME: Duration = Duration::from_nanos((1_000_000_000.0 / (1048576.0 / 17556.0)) as u64);

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let audio_subsystem = sdl_context.audio().unwrap();
    let audio_device = audio_subsystem.open_queue(None, &AudioSpecDesired{
        freq: Some(44100),
        channels: Some(2),
        samples: None,
    }).unwrap();
    let window = video_subsystem.window("oxideboy", 320, 288)
        .position_centered()
        .opengl()
        .build()
        .unwrap();
    let mut canvas = window.into_canvas().build().unwrap();
    let texture_creator = canvas.texture_creator();
    let mut texture = texture_creator.create_texture_streaming(
        PixelFormatEnum::ARGB8888, 160, 144).unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut gb_buffer = [0; gameboy::SCREEN_SIZE];

    let mut limit = false;
    let mut paused = false;
    let print_fps = env::var("PRINT_FPS").ok().unwrap_or(String::from("0")) == "1";
    let mut gameboy = gameboy::GameboyContext::new(rom);

    if !(env::var("RUN_BOOTROM").ok().unwrap_or(String::from("0")) == "1") {
        gameboy.skip_bootrom();
    }

    audio_device.resume();

    let start = Instant::now();
    let mut next_frame = FRAME_TIME;

    let mut update_fb = |gb_buffer: &[u32]| {
        texture.with_lock(None, |buffer: &mut [u8], _: usize| {
            let buffer = unsafe { slice::from_raw_parts_mut(buffer.as_ptr() as *mut u32, 160*144) };
            buffer.copy_from_slice(&gb_buffer[..]);
        }).unwrap();

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
            if print_fps { println!("FPS: {}", frames); }
            fps_timer = Instant::now();
            frames = 0;
        }

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} 
                | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                Event::KeyDown {keycode: Some(Keycode::Up), ..} => { gameboy.state.joypad.up = true; }
                Event::KeyDown {keycode: Some(Keycode::Down), ..} => { gameboy.state.joypad.down = true; }
                Event::KeyDown {keycode: Some(Keycode::Right), ..} => { gameboy.state.joypad.right = true; }
                Event::KeyDown {keycode: Some(Keycode::Left), ..} => { gameboy.state.joypad.left = true; }
                Event::KeyDown {keycode: Some(Keycode::A), ..} => { gameboy.state.joypad.a = true; }
                Event::KeyDown {keycode: Some(Keycode::S), ..} => { gameboy.state.joypad.b = true; }
                Event::KeyDown {keycode: Some(Keycode::Return), ..} => { gameboy.state.joypad.start = true; }
                Event::KeyDown {keycode: Some(Keycode::RShift), ..} => { gameboy.state.joypad.select = true; }
                Event::KeyUp {keycode: Some(Keycode::Up), ..} => { gameboy.state.joypad.up = false; }
                Event::KeyUp {keycode: Some(Keycode::Down), ..} => { gameboy.state.joypad.down = false; }
                Event::KeyUp {keycode: Some(Keycode::Right), ..} => { gameboy.state.joypad.right = false; }
                Event::KeyUp {keycode: Some(Keycode::Left), ..} => { gameboy.state.joypad.left = false; }
                Event::KeyUp {keycode: Some(Keycode::A), ..} => { gameboy.state.joypad.a = false; }
                Event::KeyUp {keycode: Some(Keycode::S), ..} => { gameboy.state.joypad.b = false; }
                Event::KeyUp {keycode: Some(Keycode::Return), ..} => { gameboy.state.joypad.start = false; }
                Event::KeyUp {keycode: Some(Keycode::RShift), ..} => { gameboy.state.joypad.select = false; }

                Event::KeyUp {keycode: Some(Keycode::L), ..} => {
                    limit = !limit;
                    audio_device.clear();
                }
                Event::KeyUp {keycode: Some(Keycode::P), ..} => { paused = !paused; }
                Event::KeyUp {keycode: Some(Keycode::M), ..} => {
                    if audio_device.status() == AudioStatus::Playing {
                        audio_device.clear();
                        audio_device.pause();
                    } else {
                        audio_device.resume();
                    }
                }
                _ => {}
            }
        }

        if !paused {
            gameboy.run_frame();
            gb_buffer.copy_from_slice(gameboy.state.ppu.framebuffer());

            let mut samples = &gameboy.state.apu.sample_queue[..];
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
        } else {
            if start.elapsed() >= next_frame {
                update_fb(&gb_buffer);
                next_frame += FRAME_TIME;
            }
        }

        frames += 1;
    }

    println!();
    println!("Executed a total of {} clock cycles", gameboy.cycle_count);
    println!();

    Ok(())
}
