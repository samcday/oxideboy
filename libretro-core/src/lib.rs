extern crate libretro_backend;
extern crate gameboy;

use std::slice;
use std::io::prelude::*;
use std::fs::File;
use libretro_backend::*;

struct OxideboyCore {
    core: Option<gameboy::GameboyContext>,
    game_data: Option<GameData>,
    audio_frame: Vec<i16>,
}

impl Default for OxideboyCore {
    fn default() -> Self {
        OxideboyCore{core: None, game_data: None, audio_frame: Vec::new()}
    }
}

impl Core for OxideboyCore {
    fn info() -> CoreInfo {
        CoreInfo::new("Oxideboy", env!("CARGO_PKG_VERSION")).supports_roms_with_extension("gb")
    }

    fn on_load_game(&mut self, game_data: GameData) -> LoadGameResult {
        if game_data.is_empty() {
            return LoadGameResult::Failed(game_data);
        }

        let mut core = if let Some(data) = game_data.data() {
            let mut rom = Vec::new();
            rom.extend_from_slice(data);
            gameboy::GameboyContext::new(gameboy::Model::DMG, rom)
        } else if let Some(path) = game_data.path() {
            let mut f = File::open(path).unwrap();
            let mut rom = Vec::new();
            f.read_to_end(&mut rom).unwrap();
            gameboy::GameboyContext::new(gameboy::Model::DMG, rom)
        } else {
            unreachable!();
        };

        core.skip_bootrom();
        self.core = Some(core);

        self.game_data = Some(game_data);

        LoadGameResult::Success(AudioVideoInfo::new()
            .video(160, 144, gameboy::FRAME_RATE, PixelFormat::ARGB8888)
            .audio(44100.0))
    }

    fn on_unload_game(&mut self) -> GameData {
        self.game_data.take().unwrap()
    }

    fn on_run(&mut self, handle: &mut RuntimeHandle) {
        match self.core.as_mut() {
            Some(core) => {
                {
                    let mut joypad = &mut core.state.joypad;
                    joypad.up = handle.is_joypad_button_pressed(0, JoypadButton::Up);
                    joypad.down = handle.is_joypad_button_pressed(0, JoypadButton::Down);
                    joypad.left = handle.is_joypad_button_pressed(0, JoypadButton::Left);
                    joypad.right = handle.is_joypad_button_pressed(0, JoypadButton::Right);
                    joypad.a = handle.is_joypad_button_pressed(0, JoypadButton::A);
                    joypad.b = handle.is_joypad_button_pressed(0, JoypadButton::B);
                    joypad.start = handle.is_joypad_button_pressed(0, JoypadButton::Start);
                    joypad.select = handle.is_joypad_button_pressed(0, JoypadButton::Select);
                }

                core.run_frame();

                let buffer = unsafe { slice::from_raw_parts(core.state.ppu.framebuffer().as_ptr() as *const u8, 160*144*4) };
                handle.upload_video_frame(buffer);

                self.audio_frame.clear();
                for sample in &core.state.apu.sample_queue[..] {
                    self.audio_frame.push((sample * 32767.0) as i16);
                }
                while self.audio_frame.len() < ((44100.0 / gameboy::FRAME_RATE).ceil() as usize) * 2 {
                    let extra = self.audio_frame[self.audio_frame.len() - 1];
                    self.audio_frame.push(extra);
                }
                handle.upload_audio_frame(&self.audio_frame);
            }
            None => unreachable!()
        }
    }

    fn on_reset(&mut self) {
        panic!("Unimplemented");
    }
}

libretro_core!(OxideboyCore);
