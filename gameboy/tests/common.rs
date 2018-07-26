extern crate image;

use std::env;

pub fn compare_framebuffer<F>(framebuf: &[u32], expected: &[u8], color_conv: F)
    where F: Fn(u32) -> u32
{
    let expected_img = image::load_from_memory(expected).unwrap();
    let expected_img = expected_img.to_rgb();

    for y in 0..144 {
        for x in 0..160 {
            let expected_rgb = expected_img.get_pixel(x as u32, y as u32);
            let expected_rgb = 0xFF000000 | ((expected_rgb[0] as u32) << 16) | ((expected_rgb[1] as u32) << 8) | (expected_rgb[2] as u32);

            let framebuffer_pix = framebuf[(y * 160) + x];

            // Our palette differs from reference images, so we convert the colours here.
            let expected_rgb = color_conv(expected_rgb);

            if framebuffer_pix != expected_rgb {
                // Save the framebuffer to a temp file for examination.
                let mut path = env::temp_dir();
                path.push("result.png");
                let mut framebuf_image = vec![0u8; 160*144*3];
                for i in 0..160*144 {
                    framebuf_image[i*3] = ((framebuf[i] & 0xFF0000) >> 16) as u8;
                    framebuf_image[i*3+1] = ((framebuf[i] & 0xFF00) >> 8) as u8;
                    framebuf_image[i*3+2] = (framebuf[i] & 0xFF) as u8;
                }
                image::save_buffer(path.clone(), &framebuf_image[..], 160, 144, image::ColorType::RGB(8)).unwrap();

                panic!("Pixel {},{} does not match. Expected {:X}, got {:X}. Result saved to {}", x, y, expected_rgb, framebuffer_pix, path.to_str().unwrap());
            }
        }
    }
}
