# Audio sample rate in Hz (e.g.,8000, 16000, 44100, 48000)
int16 sample_rate

# Number of audio channels (1=mono, 2=stereo)
uint8 channels

# Bit depth per sample (e.g., 8, 16, 24)
uint8 bit_depth

# Raw PCM audio data (consider endianness for multi-byte formats)
std_msgs/ByteMultiArray audio_data
