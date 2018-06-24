extern crate gameboy;
extern crate ratelimit;
extern crate sdl2;

use std::io::prelude::*;
use std::fs::File;
use std::env;
use std::time::{Duration, Instant};
use std::rc::Rc;
use std::cell::RefCell;

use sdl2::audio::{AudioSpecDesired};
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
    let audio_device = Rc::new(RefCell::new(audio_subsystem.open_queue(None, &AudioSpecDesired{
        freq: Some(44100),
        channels: Some(2),
        samples: None,
    }).unwrap()));
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

    let mut limit = false;
    let mut ratelimit = ratelimit::Builder::new()
        .capacity(1).quantum(1)
        .interval(Duration::new(0, 16660000))
        .build();

    let print_serial = env::var_os("PRINT_SERIAL").map(|s| &s == "1").unwrap_or(false);

    let gb_buffer = Rc::new(RefCell::new([0; gameboy::SCREEN_SIZE]));

    let cb_gb_buffer = Rc::clone(&gb_buffer);
    let mut video_cb = move |buf: &[u32]| {
        cb_gb_buffer.borrow_mut().copy_from_slice(buf);
    };

    let cb_audio_device = Rc::clone(&audio_device);
    let mut sound_cb = move |(l, r)| {
        cb_audio_device.borrow_mut().clear();
        cb_audio_device.borrow_mut().queue(&[l, r]);
    };

    let mut serial_cb = |sb| {
        if print_serial {
            std::io::stdout().write(&[sb]).unwrap();
            std::io::stdout().flush().unwrap();
        }
    };

    let mut paused = false;
    let mut gameboy = gameboy::CPU::new(rom, &mut video_cb, &mut sound_cb, &mut serial_cb);

    audio_device.borrow_mut().resume();

    // gameboy.breakpoint = 0x681;

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
                Event::KeyUp {keycode: Some(Keycode::P), ..} => { paused = !paused; }
                _ => {}
            }
        }

        if !paused {
            delta = gameboy.run_frame(delta);

            if now.elapsed().subsec_nanos() > 16666666 {
                now = Instant::now();

                texture.with_lock(None, |buffer: &mut [u8], pitch: usize| {
                    let gb_buffer = gb_buffer.borrow();
                    for y in 0..144 {
                        for x in 0..160 {
                            let offset = y*pitch + (x*3);
                            buffer[offset] = gb_buffer[y*160+x] as u8;
                            buffer[offset + 1] = gb_buffer[y*160+x] as u8;
                            buffer[offset + 2] = gb_buffer[y*160+x] as u8;
                        }
                    }
                }).unwrap();

                canvas.clear();
                canvas.copy(&texture, None, Some(Rect::new(0, 0, 320, 288))).unwrap();
                canvas.present();
            }
        }

        if gameboy.breakpoint_hit {
            break 'running;
        }
    }

    println!();
    println!("Sigh: {}", gameboy.pc());
    println!("Executed a total of {} clock cycles", gameboy.cycle_count);
    println!();

    Ok(())
}
