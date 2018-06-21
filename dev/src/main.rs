#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]
extern crate gameboy;
extern crate ratelimit;
extern crate sdl2;

use std::io;
use std::io::prelude::*;
use std::fs::File;
use std::thread;
use std::sync::{Mutex, Arc, mpsc};
use std::env;
use std::time::{Duration, Instant};
use std::rc::Rc;
use std::cell::RefCell;

use sdl2::audio::{AudioSpecDesired, AudioQueue};
use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

type Result<T> = std::result::Result<T, Box<std::error::Error>>;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
    let audio_subsystem = sdl_context.audio().unwrap();
    let audio_device: AudioQueue<f32> = audio_subsystem.open_queue(None, &AudioSpecDesired{
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
    let mut texture = Rc::new(RefCell::new(texture_creator.create_texture_streaming(
        PixelFormatEnum::RGB24, 160, 144).unwrap()));
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut limit = false;
    let mut ratelimit = ratelimit::Builder::new()
        .capacity(1).quantum(1)
        .interval(Duration::new(0, 16660000))
        .build();

    let print_serial = env::var_os("PRINT_SERIAL").map(|s| &s == "1").unwrap_or(false);

    let video_cb_texture = Rc::clone(&texture);
    let mut video_cb = move |buf: &[u32]| {
        video_cb_texture.borrow_mut().with_lock(None, |buffer: &mut [u8], pitch: usize| {
            for y in 0..144 {
                for x in 0..160 {
                    let offset = y*pitch + (x*3);
                    buffer[offset] = buf[y*160+x] as u8;
                    buffer[offset + 1] = buf[y*160+x] as u8;
                    buffer[offset + 2] = buf[y*160+x] as u8;
                }
            }
        }).unwrap();
    };

    let mut gameboy = gameboy::CPU::new(rom, &mut video_cb);

    audio_device.resume();

    gameboy.breakpoint = 0x681;

    let mut delta = 0;

    let mut now = Instant::now();
    'running: loop {
        if limit {
            ratelimit.wait();
        }

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} 
                | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                Event::KeyDown {keycode: Some(Keycode::Up), ..} => { gameboy.joypad().up = true; }
                Event::KeyDown {keycode: Some(Keycode::Down), ..} => { gameboy.joypad().down = true; }
                Event::KeyDown {keycode: Some(Keycode::Right), ..} => { gameboy.joypad().right = true; }
                Event::KeyDown {keycode: Some(Keycode::Left), ..} => { gameboy.joypad().left = true; }
                Event::KeyDown {keycode: Some(Keycode::A), ..} => { gameboy.joypad().a = true; }
                Event::KeyDown {keycode: Some(Keycode::S), ..} => { gameboy.joypad().b = true; }
                Event::KeyDown {keycode: Some(Keycode::Return), ..} => { gameboy.joypad().start = true; }
                Event::KeyDown {keycode: Some(Keycode::RShift), ..} => { gameboy.joypad().select = true; }
                Event::KeyUp {keycode: Some(Keycode::Up), ..} => { gameboy.joypad().up = false; }
                Event::KeyUp {keycode: Some(Keycode::Down), ..} => { gameboy.joypad().down = false; }
                Event::KeyUp {keycode: Some(Keycode::Right), ..} => { gameboy.joypad().right = false; }
                Event::KeyUp {keycode: Some(Keycode::Left), ..} => { gameboy.joypad().left = false; }
                Event::KeyUp {keycode: Some(Keycode::A), ..} => { gameboy.joypad().a = false; }
                Event::KeyUp {keycode: Some(Keycode::S), ..} => { gameboy.joypad().b = false; }
                Event::KeyUp {keycode: Some(Keycode::Return), ..} => { gameboy.joypad().start = false; }
                Event::KeyUp {keycode: Some(Keycode::RShift), ..} => { gameboy.joypad().select = false; }

                Event::KeyUp {keycode: Some(Keycode::L), ..} => { limit = !limit; }
                _ => {}
            }
        }

        {
            let newdelt = gameboy.run_frame(delta);

            if now.elapsed().subsec_nanos() > 16666666 {
                now = Instant::now();

                // audio_device.clear();
                // audio_device.queue(audio_samples);

                canvas.clear();
                canvas.copy(&texture.borrow(), None, Some(Rect::new(0, 0, 320, 288))).unwrap();
                canvas.present();
            }
        }

        if gameboy.breakpoint_hit {
            break 'running;
        }

        // if print_serial {
        //     let ser = gameboy.cpu.serial_get();
        //     if ser.is_some() {
        //       io::stdout().write(&[ser.unwrap()]).unwrap();
        //       io::stdout().flush().unwrap();
        //     }
        // }

        // if gameboy.breakpoint_hit {
        //     break 'running;
        // }
    }

    println!();
    println!("Sigh: {}", gameboy.pc());
    println!("Executed a total of {} clock cycles", gameboy.cycle_count);
    println!();

    Ok(())
}
