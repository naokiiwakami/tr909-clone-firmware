#!/usr/bin/env python3

import wave
import sys


def read_wav(file_name: str, data_name: str):
    """Reads a PCM wav file"""
    with wave.open(file_name, mode="rb") as wav_file:
        nchannels = wav_file.getnchannels()
        sampwidth = wav_file.getsampwidth()
        if nchannels != 1 or sampwidth != 1:
            print("The wav file is expected to be monaural and 8-bit but not. Check the file")
            sys.exit(1)
        nframes = wav_file.getnframes()
        sample_bytes = wav_file.readframes(nframes)
        # print(bytes.hex())
        print(f"#ifndef {data_name.upper()}_H_")
        print(f"#define {data_name.upper()}_H_\n")
        print(f"static uint8_t {data_name}[{nframes}] = {{")
        for i, sample in enumerate(sample_bytes):
            if i % 16 == 0:
                print(f"  /* {i:#4x} */ ", end="")
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
