import importlib
import time

# Hack to import the teensy module
spec = importlib.util.spec_from_file_location("teensy41", "src/teensy41/teensy41.py")
teensy41 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(teensy41)

if __name__ == "__main__":
    teensy = teensy41.Teensy41()
    total_disp1 = 0
    total_disp2 = 0
    start_time = time.time()
    while time.time() - start_time < 10:
        disp1, disp2, timestamp = teensy.get_motors_displacement_with_timestamp()
        # Uncomment these if you would like to debug
        # print(f"At teensy time {timestamp}:")
        # print(f"disp1 = {disp1}")
        # print(f"disp2 = {disp2}")
        # print("\n")
        total_disp1 += disp1
        total_disp2 += disp2
        time.sleep(0.01)
    print(f"Total displacement of Motor1 = {total_disp1}")
    print(f"Total displacement of Motor2 = {total_disp2}")
    teensy.close_device()
