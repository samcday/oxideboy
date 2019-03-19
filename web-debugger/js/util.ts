export function toPaddedHexString(num: number, len: number): string {
    const str = num.toString(16);
    return "0".repeat(len - str.length) + str;
}
