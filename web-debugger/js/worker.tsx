import('../crate/pkg').catch(err => console.error('Failed to load worker.', err));

// loadedHandlers.pause = (message) => {
//   clearInterval(tickId);
//   tickId = null;
//   postMessage({type: 'paused'});
// };

// loadedHandlers.keydown = (message) => {
//   emulator.set_joypad_state(message.key, true);
// };

// loadedHandlers.keyup = (message) => {
//   emulator.set_joypad_state(message.key, false);
// };

balls = (framebuffer, memory) => {
	postMessage({type: 'state', framebuffer, memory}, [framebuffer.buffer, memory.buffer]);
	// console.log('really doe', left.framebuffer.buffer === right[0] );
};
