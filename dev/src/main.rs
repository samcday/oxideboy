#![allow(unused_imports)]
#![allow(unused_variables)]
#![allow(unused_mut)]
extern crate lr35902;
extern crate gameboy;
extern crate ratelimit;
extern crate sdl2;

use std::io;
use std::io::prelude::*;
use std::fs::File;
use std::time::Duration;
use std::thread;
use std::sync::{Mutex, Arc, mpsc};
use std::env;

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
    let mut texture = texture_creator.create_texture_streaming(
        PixelFormatEnum::RGB24, 160, 144).unwrap();
    let mut event_pump = sdl_context.event_pump().unwrap();

    let mut limit = true;
    let mut ratelimit = ratelimit::Builder::new()
        .capacity(1) //number of tokens the bucket will hold
        .quantum(1) //add one token per interval
        .interval(Duration::new(0, 16660000)) //add quantum tokens every 1 second
        .build();

    let framebuffer = [0 as u32; 160*144];
    let print_serial = env::var_os("PRINT_SERIAL").map(|s| &s == "1").unwrap_or(false);

    let mut gameboy = gameboy::Gameboy::new(&rom)?;

            // if cpu.pc() == 0x681 {
            //     break;
            // }

    //         let cycles = cpu.run();
    //         if print_serial {
    //             let ser = cpu.serial_get();
    //             if ser.is_some() {
    //               io::stdout().write(&[ser.unwrap()]).unwrap();
    //               io::stdout().flush().unwrap();
    //             }
    //         }
    //         if cpu.is_vblank() {
    //             {
    //                 let mut emu_framebuffer = emu_framebuffer.lock().unwrap();
    //                 emu_framebuffer.copy_from_slice(&cpu.framebuffer()[..]);
    //             }
    //         }

    //         if limit { emu_ratelimit.wait_for(cycles as usize); }
    //     }

    //     println!();
    //     println!("Executed a total of {} clock cycles", cpu.cycle_count);
    //     println!();
    // });

    audio_device.resume();

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

        let (framebuffer, audio_samples) = gameboy.run_frame();
        audio_device.clear();
        audio_device.queue(audio_samples);
        
        texture.with_lock(None, |buffer: &mut [u8], pitch: usize| {
            for y in 0..144 {
                for x in 0..160 {
                    let offset = y*pitch + (x*3);
                    buffer[offset] = framebuffer[y*160+x] as u8;
                    buffer[offset + 1] = framebuffer[y*160+x] as u8;
                    buffer[offset + 2] = framebuffer[y*160+x] as u8;
                }
            }
        }).unwrap();

        canvas.clear();
        canvas.copy(&texture, None, Some(Rect::new(0, 0, 320, 288))).unwrap();
        canvas.present();

    }

    Ok(())
}
