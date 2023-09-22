
import yaml
import os
from PyQt5.QtWidgets import QApplication, QDesktopWidget, QFileDialog

def window_size_pos(w):
    d = {}
    d['width'] = w.size().width()
    d['height'] = w.size().height()
    d['x'] = w.pos().x()
    d['y'] = w.pos().y()
    return d

def save_window_pos(window, key_name, file_name=''):
    if  file_name == '':
        file_name, _ = QFileDialog.getSaveFileName(window, "Save window position","","All Files (*);;Text Files (*.txt)")
    # if the file exists, we amend it
    current = {}
    if os.path.exists(file_name):
        current = yaml.full_load(open(file_name))
    current[key_name] = window_size_pos(window)
    with open(file_name, 'w') as file:
        yaml.dump(current, file)
    print(f"Saving the position of {key_name} in  {file_name}")
    return file_name