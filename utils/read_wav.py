#!/usr/bin/env python3

import wave
import sys


def read_wav(file_name: str, data_name: str):
    """Reads a PCM wav file and dump the samples as an array of uint8_t in C for AVR"""
    with wave.open(file_name, mode="rb") as wav_file:
        nchannels = wav_file.getnchannels()
        sampwidth = wav_file.getsampwidth()
        if nchannels != 1 or sampwidth != 1:
            print("The wav file is expected to be monaural and 8-bit but this file is not.")
            sys.exit(1)
        nframes = wav_file.getnframes()
        sample_bytes = wav_file.readframes(nframes)
        # print(bytes.hex())
        print(f"#ifndef {data_name.upper()}_H_")
        print(f"#define {data_name.upper()}_H_\n")
        print("#include <avr/pgmspace.h>\n")
        print(f"const uint8_t {data_name}[{nframes}] PROGMEM = {{")
        for i, sample in enumerate(sample_bytes):
            # Do sanity check on the sample. The value must not have two LSB bits.
            if sample & 0x3 != 0:
                # raise Exception(f"sample [{i}] has dirty two LSBs ({sample})")
                sample = sample & 0xFC
            if i % 16 == 0:
                print(f"  /* {i:#04x} */ ", end="")
            print(f" {sample:#x},", end="")
            if i % 16 == 15:
                print("\n", end="")
        print("};")
        print(f"\n#endif  /* {data_name.upper()}_H_ */")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: read_wav.py <wav_file_name>")
        sys.exit(1)
    read_wav(sys.argv[1], sys.argv[2])
