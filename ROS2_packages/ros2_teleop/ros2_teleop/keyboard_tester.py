from pynput import keyboard
from pynput.keyboard import Key


pressed_keys = {
    Key.esc : False,
    Key.enter : False,
    Key.shift : False,
    Key.ctrl : False,
    "q" : False,
    "w" : False,
    "e" : False,
    "a" : False,
    "s" : False,
    "d" : False,
    "z" : False,
    "x" : False,
    "c" : False,
    Key.up : False,
    Key.down : False,
    Key.right : False,
    Key.left : False,
}
def on_press(key):
    try:
        # alphanumerical characters
        lookup = key.char
    except AttributeError:
        # special characters
        lookup = key

    if lookup in pressed_keys:
        pressed_keys[lookup]=True
        print("pressed:", pressed_keys)

def on_release(key):
    try:
        # alphanumerical characters
        lookup = key.char
    except AttributeError:
        # special characters
        lookup = key
        
    if lookup in pressed_keys:
        pressed_keys[lookup]=False
        print("released:", pressed_keys)

# Collect events until released
with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
    listener.join()
"""
# ...or, in a non-blocking fashion:
listener = k.keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()
"""
