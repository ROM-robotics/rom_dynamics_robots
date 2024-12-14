import tkinter as tk
from tkinter import messagebox
from cefpython3 import cefpython as cef
import threading
import sys


def test_thread(frame, rect):
    sys.excepthook = cef.ExceptHook
    window_info = cef.WindowInfo(frame.winfo_id())
    window_info.SetAsChild(frame.winfo_id(), rect)
    cef.Initialize()
    #browser = cef.CreateBrowserSync(window_info, url='https://google.com')
    browser = cef.CreateBrowserSync(window_info, url='http://localhost:5000')
    cef.MessageLoop()


def on_closing():
    print('closing')
    root.destroy()


def on_resize(event):
    # Get current window width and height
    width = root.winfo_width()
    height = root.winfo_height()

    # Adjust the left frame (browser) and the right frame (buttons) dynamically
    browser_width = int(width * 0.75)  # 75% of window width
    button_width = int(width * 0.25)   # 25% of window width
    browser_height = height - scrollbar_height  # Reserve space for the horizontal scrollbar

    # Update geometry of frames
    browser_frame.config(width=browser_width, height=browser_height)
    buttons_frame.config(width=button_width, height=browser_height)
    
    # Remove any padding to eliminate gaps
    browser_frame.pack_configure(fill="both", expand=True)
    buttons_frame.pack_configure(fill="both", expand=True)
    
    exit_button.pack(side='bottom', anchor='se', padx=20, pady=20)

    # Update the rect for CEF browser
    global rect
    rect = [0, 0, browser_width, browser_height]

    # Restart the CEF browser in the new size if needed  //removed thread
    # if not thread.is_alive():
    #     thread = threading.Thread(target=test_thread, args=(browser_frame, rect))
    #     thread.start()


root = tk.Tk()
root.title("Funviz")

# Initial fullscreen-like window based on screen size
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()
root.geometry(f"{screen_width}x{screen_height}")

# Handle window close event
root.protocol('WM_DELETE_WINDOW', on_closing)

# Create the main frame to hold both browser and buttons frames
frame = tk.Frame(root)
frame.pack(fill='both', expand=True)

# Frame for the browser (left side, initially taking 75% width)
browser_frame = tk.Frame(frame)
browser_frame.pack(side='left', fill='both', expand=True)

# Add a horizontal scrollbar (slider) at the bottom of the browser frame
scrollbar_height = 20  # Height reserved for the scrollbar
# scrollbar_horizontal = tk.Scrollbar(browser_frame, orient="horizontal")
# scrollbar_horizontal.pack(side="bottom", fill="x")s

# Frame for buttons (right side, initially taking 25% width)
buttons_frame = tk.Frame(frame, bg='gray', width=int(screen_width * 0.25))
buttons_frame.pack(side='right', fill='both', expand=True)

# Add buttons to the right frame
tk.Button(buttons_frame, text='Topics Lists',
          command=lambda: messagebox.showinfo('Running Topics in ROS2 System', 'Showing Topics')).pack(side='top', padx=20, pady=10)

# Exit button placed at the bottom right
exit_button = tk.Button(buttons_frame, text='Exit', command=on_closing)
exit_button.pack(side='bottom', anchor='se', padx=20, pady=20)

# Set initial rect for the CEF browser (taking the left side, 75% width minus scrollbar height)
rect = [0, 0, int(screen_width * 0.80), screen_height - scrollbar_height]
print('browser: ', rect[2], 'x', rect[3])

# Start the CEF browser thread
thread = threading.Thread(target=test_thread, args=(browser_frame, rect))
thread.start()

# Bind the resize event to dynamically adjust the layout
root.bind('<Configure>', on_resize)

# Start the Tkinter event loop
root.mainloop()
