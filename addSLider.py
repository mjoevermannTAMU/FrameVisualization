import tkinter as tk
import time


def a_function(count=0):
    print(f'the angle has changed to {count} degrees!')


def run():
    # create a slider to control encoder angle
    sliderWindow = tk.Tk()
    sliderWindow.geometry('300x70')
    sliderWindow.title('Encoder Value Slider')
    encoder_value = tk.DoubleVar()
    val = tk.Scale(sliderWindow, from_=-30, to=30, variable=encoder_value, orient='horizontal', label='Set Rotation (deg)', length=300, command=a_function)
    val.pack()
    while True:
        sliderWindow.update_idletasks()
        sliderWindow.update()
        time.sleep(0.05)
run()