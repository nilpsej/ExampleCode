try:
    from pynput.keyboard import Key, Controller
except:
    os.system('python -m pip install pynput')
    from pynput.keyboard import Key, Controller

sim_keyboard = Controller()
import os, time

hz = 0
hz = int(input("Please Enter Frequency:"))

if(hz != 10 and hz != 20 and hz != 50 and hz != 250):
    hz = input("Please choose a frequency from the following, 10, 20, 50 or 250hz:")
    
file = open('hardware.yaml', 'r')

file_contents = file.read()
position = file_contents.find("loop_hz:") + 9

contents = []
for x in file_contents:
    contents += [x]

for x in range(len(str(hz))):
    contents[position+x] = str(hz)[x]

write_to_file = ''
for x in contents:
    write_to_file += x
print(write_to_file)
file = open('hardware.yaml', 'w')
file.write(write_to_file)
file.close()


if(hz == 10):
   hex_val = '64'
elif(hz == 20):
    hex_val = '32'
elif(hz == 50):
    hex_val = '14'
else:
    hex_val = '04'

text1 = "cansend can0 60"
text2 = "#2B001805"
text3 = "000000"

text4 = "#2310100173617665"


sim_keyboard.press(Key.cmd)
sim_keyboard.release(Key.cmd)
time.sleep(0.5)
sim_keyboard.type('cmd')
time.sleep(0.5)
sim_keyboard.press(Key.enter)
sim_keyboard.release(Key.enter)
time.sleep(1)

def initalise():

    sim_keyboard.type("sudo ip link set can0 up type can bitrate 250000\n")

from pynput import keyboard

def on_press(key):
    if key == keyboard.Key.shift_r:
        # Stop listener
        return False

# Collect events until released

def wait_for_shift():
    with keyboard.Listener(
        on_press=on_press) as listener:
        listener.join()

for x in range(5):
    print(x)
    time.sleep(1)
    initalise()
    time.sleep(1)
    sim_keyboard.type(text1+str(x+1)+text2+hex_val+text3+'\n')
    print("Press shift after response")
    wait_for_shift()
    sim_keyboard.type(text1+str(x+1)+text4+'\n')
    print("Move to next encoder, then press shift")
    wait_for_shift()

    
    
    
