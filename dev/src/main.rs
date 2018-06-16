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
use std::sync::{Mutex, Arc};

use sdl2::pixels::PixelFormatEnum;
use sdl2::rect::Rect;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;

fn main() -> io::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let mut f = File::open(&args[1])?;
    let mut rom = Vec::new();
    f.read_to_end(&mut rom)?;

    let sdl_context = sdl2::init().unwrap();
    let video_subsystem = sdl_context.video().unwrap();
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

    let mut ratelimit = ratelimit::Builder::new()
        .capacity(1) //number of tokens the bucket will hold
        .quantum(1) //add one token per interval
        .interval(Duration::new(0, 16660000)) //add quantum tokens every 1 second
        .build();

    // let arr: [u8; 160*144] = ;
    let framebuffer = Arc::new(Mutex::new([0 as u32; 160*144]));

    let emu_framebuffer = Arc::clone(&framebuffer);
    thread::spawn(move || {
        let mut cart = gameboy::cartridges::MBC1Cart::new(&rom);
        let cpu = &mut lr35902::CPU::new(&mut cart);

        loop {
            if cpu.pc() == 0x681 {
                break;
            }

            cpu.run();
            let ser = cpu.serial_get();
            if ser.is_some() {
              io::stdout().write(&[ser.unwrap()]).unwrap();
              io::stdout().flush().unwrap();
            }
            if cpu.is_vblank() {
                {
                    let mut emu_framebuffer = emu_framebuffer.lock().unwrap();
                    emu_framebuffer.copy_from_slice(&cpu.framebuffer()[..]);
                }
            }
        }

        println!();
        println!("Executed a total of {} clock cycles", cpu.cycle_count);
        println!();
    });

    'running: loop {
        ratelimit.wait();

        for event in event_pump.poll_iter() {
            match event {
                Event::Quit {..} 
                | Event::KeyDown { keycode: Some(Keycode::Escape), .. } => {
                    break 'running
                },
                _ => {}
            }
        }

        {
            let framebuffer = framebuffer.lock().unwrap();
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
        }

        canvas.clear();
        canvas.copy(&texture, None, Some(Rect::new(0, 0, 320, 288))).unwrap();
        canvas.present();
    }

    Ok(())
}
