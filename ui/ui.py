import os
import re
import sys
import math
import time
import hashlib
import argparse
from pyqtgraph import PlotWidget, mkPen, mkColor, LegendItem
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from data import load_all, decode, encode_html

OPEN_LEGEND = False
HEIGHT_RATIO = 0.65
parser = argparse.ArgumentParser(description="")
parser.add_argument('-t', '--test', action='store_true', help='Run test')
args = parser.parse_args()

class MousePlotWidget(PlotWidget) :
    obs_list = {}

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        bct_pos = self.plotItem.vb.mapSceneToView(event.pos())
        print(bct_pos)
        if event.buttons() == Qt.LeftButton:
            if self.obs_list:
                min_id = 0
                min_distance = 1e6
                for id, (x, y) in self.obs_list.items():
                    dist = ((x - bct_pos.x())**2 +(y - bct_pos.y())**2) ** 0.5
                    if dist < min_distance:
                        min_distance = dist
                        min_id = id
                if min_distance < 4:
                    color = id_to_rgb(min_id)
                    style = {'color': "#{:2x}{:2x}{:2x}".format(color[0], color[1], color[2])}
                    self.setTitle(min_id, **style)

def id_to_rgb(id):
    #生成哈希值
    if id == 0:
        return (0, 0, 255)
    hash = hashlib.sha256(str(id).encode('utf-8'))
    color = hash.hexdigest()[:6]
    #将哈希値转换为RGB顔色
    def trans(start, end):
        return int(float(int(color[start:end], 16)) / 255 * (200 - 70) + 70)
    r = trans (0, 2)
    g = trans (2, 4)
    b = trans (4, 6)
    color = (r, g, b, 255)
    return color

class DataLoader(QThread):
    send_signal = pyqtSignal(str)

    def __init__(self):
        QThread.__init__(self)
        if args.test:
            print("****************************Use Test *******************************")
            self.file_path = os.path.dirname(os.path.abspath(__file__)) + "/test.txt"
        else:
            self.file_path = os.path.dirname(os.path.abspath(__file__)) + "/mm.txt"

        self.reset()
        self.time = time.perf_counter()
        self.check_new = CheckNew(self.file_path)
        self.check_new.send_signal.connect(self.check_new_callback)
        self.check_new.start()
    
    def get_data(self):
        # need check
        if self.check_data_wrong():
            print("wrong data")
            self.data = load_all(self.file_path)
        return self.data
    
    def check_data_wrong(self):
        return False

    def check_new_callback(self, info):
        if info == "restart":
            self.reset()
    
    def reset(self):
        self.ready = False
        self.position = 0
        self.data = {}
        self.line = ''
        self.ready = True
    
    def run (self):
        has_new = False
        while True:
            if not os.path.exists(self.file_path):
                print("wait file")
                time.sleep(0.5)
                continue

            with open(self.file_path, 'rb') as file: #打汗文件
                file.seek(self.position, 1)
                lines = file.readlines()
                for i, line in enumerate(lines):
                    data = line.decode()
                    self.line += data
                    if self.line[-1] == '*' or self.line[-1] == '\n':
                        self.line = self.line.strip()[:-1]
                        if len(self.line) == 0:
                            print("empty")
                            continue
                        frame_num, scene_idx = self.__decode(self.line)
                        has_new = True
                        self.time = time.perf_counter()
                        self.line = ''
                current_position = file.tell()
                if self.ready or (time.perf_counter() - self.time > 0.5 and has_new):
                    has_new = False
                    print ("decode time:", time.perf_counter() - self.time)
                    # self.time = time.perf counter()
                    self.send_signal.emit("update")
                    self.ready = False
                
                self.position = current_position
    
    def __decode(self, line):
        elements = re.findall('frame_id\[(.*?)\]\[(.*?)\]', line[:20])[:1][0]
        frame_num, scene_idx = map(int, elements)
        if frame_num not in self.data:
            self.data[frame_num] = {}
        if scene_idx not in self.data[frame_num]:
            self.data[frame_num][scene_idx] = ()
        self.data[frame_num][scene_idx] = decode(line)
        print ("decode:", frame_num, scene_idx, self.data.keys(), self.data[frame_num].keys())
        return frame_num, scene_idx

class CheckNew(QThread):
    send_signal = pyqtSignal(str)
    def __init__(self, file_path):
        QThread.__init__(self)
        self.file_path = file_path
        self.last_frame_num = -1

    def run(self):
        while True:
            if not os.path.exists(self.file_path):
                print("wait file")
                time.sleep (0.5)
                continue
        
            start = time.perf_counter()
            try:
                with open(self.file_path, 'rb')as f: #打开文件
                    first_line = f.readline() #读第一行
                    off = -50 #没置偏移量
                    while True:
                        f.seek(off, 2) #seek（off,2）表示文件指针：从文件末尾（2）开始向前50个字符（-50）
                        lines = f.readlines() #读取文件指针范围内所有行
                        if len(lines) >= 2: #判断是否最后至少有两行，这样保证了最后一行是完整的
                            last_line = lines[-1] #取最后一行
                            break
                        off *= 2
                    last_line = last_line.decode()
            except:
                with open(self.file_path, 'r') as t: #打井文件
                    last_line = t.readlines()
                    if type(last_line) == list:
                        last_line = last_line[-1]
            frame_num = self.get_frame_num(last_line)
            # print(frame_num, self.last_frame_num)
            if frame_num < self.last_frame_num:
                print("!!!!!!!!!!!!!!!!!!!!!!!!! restart case")
                self.send_signal.emit('restart')
            if frame_num > self.last_frame_num:
                # print ("yes")
                print("!!!!!!!!!!!!!!!!!!!!!!!!! newline")
                self.send_signal.emit('new')
        
            self.last_frame_num = frame_num

            end = time.perf_counter()
            duration = end - start
            # print("check new: ", duration)
            if end - start < 0.2:
                time.sleep(0.2 - duration)
    
    def get_frame_num(self, line):
        try:
            frame_num = int(re.findall('frame_id\[(.*?)\]', line[:20])[0])
            return frame_num
        except:
            print(line[:20], re.findall('frame_id\[(.*?)\]', line[:20]))
            return self.last_frame_num

class MyUI(QWidget):
    def __init__(self, width, height):
        QWidget.__init__(self)

        print("loading...")
        self.dataloader = DataLoader()
        self.dataloader.send_signal.connect(self.dataloader_callback)
        self.dataloader.start()
        print("load done")

        self.desktop_width = width
        self.desktop_height = height
        self.desktop_height = 1900
        self.sub_window_size = (self.desktop_height * HEIGHT_RATIO) / 2
        # self.resize(width, height)

        self.scene_num = self.total_scenec_num = 2
        self.step = self.total_step = 0
        self.frame = self.total_frame = 0
        self.delete_dict = []
        self.scene_blank_list = []
        self.plot_list = []
        self.plot_frame_step_id_polygon = {}
        self.scroll_list = []
        self.data = {}

        self.update_flag = False

        self.layout = QVBoxLayout() # 从上到下竖直布局
        self.setLayout(self.layout)
        self.layout.addStretch()
        self.reset()
        self.layout.addStretch()

    def dataloader_callback(self, info):
        print("callback")
        if self.update_flag: return
        data = {}
        while data == {}:
            data = self.dataloader.get_data()
        self.data = data
        self.frame = self.total_frame = max(self.data.keys())
        self.total_scene_num = max(self.data[self.frame].keys()) + 1
        self.step = self.total_step = 0
        for scene in self.data[self.frame].values():
            for obs in scene.values():
                self.total_step = max(self.total_step, len(obs['traj']) - 1)
        self.update_vis()
    
    def update_text(self):
        for scene_blank in self.scene_blank_list:
            scene_blank.setHtml ("")
        for i in range(min(len(self.scene_blank_list), self.total_scene_num)):
            self.scene_blank_list[i].setHtml(encode_html(self.data[self.frame][i]))
    
    def update_status(self):
        print ("frame:", self.frame, self.total_frame)
        print ("scene:", self.scene_num, self.total_scene_num)
        print ("step:", self.step, self.total_step)

        self.frame_slider.blockSignals(True)
        self.frame_slider.setMaximum(self.total_frame)
        self.frame_slider.setValue(self.frame)
        self.frame_slider.blockSignals(False)

        self.step_slider.blockSignals (True)
        self.step_slider.setMaximum(self.total_step)
        self.step_slider.setValue(self.step)
        self.step_slider.blockSignals(False)

        self.frame_label.setText(f"frame: {self.frame}/{self.total_frame}")
        self.step_label.setText(f"step: {self.step}/{self.total_step}")
        self.scene_label.setText(f"scene: {self.scene_num}/{self.total_scene_num}")
    
    def draw_polygon(self, x, y, theta, x0, y0, theta0, polygon) :
        x_list = []
        y_list = []
        rtheta = theta - theta0
        for pt in polygon:
            shift_x = pt[0] + x - x0
            shift_y = pt[1] + y - y0

            rotate_x = math.cos(rtheta) * (shift_x - x) - math.sin(rtheta) * (shift_y - y) + x
            rotate_y = math.sin(rtheta) * (shift_x - x) + math.cos(rtheta) * (shift_y - y) + y

            x_list.append(rotate_x)
            y_list.append(rotate_y)

        x_list.append(x_list[0])
        y_list.append(y_list[0])
        return x_list, y_list
    
    def update_vis(self, update_step_only=False):
        self.update_flag = True

        self.total_scene_num = max(self.data[self.frame].keys()) + 1

        self.update_status()

        start = time.perf_counter()

        print( "update", self.frame, self.step, self.total_scene_num)
        if update_step_only:
            for i in range(min(len(self.plot_list), self.total_scene_num)):
                for id, id_data in self.data[self.frame][i].items():
                    color = id_to_rgb(id)
                    traj = id_data['traj']
                    step = min(self.step, len(traj) - 1)
                    x0, y0, theta0 = traj[0][1:]
                    x, y, theta = traj[step][1:]
                    polygon_x, polygon_y = self.draw_polygon(x, y, theta, x0, y0, theta0, id_data['polygon'])
                    plt = self.plot_list[i]
                    plt.obs_list[id] = (x, y)
                    try:
                        self.plot_frame_step_id_polygon[self.frame][i][id].setData(polygon_x, polygon_y)
                    except:
                        plt = self.plot_list[i]
                        plot = plt.plot(polygon_x, polygon_y, pen=mkPen(color=color, width=3), name=str(id))
                        if self.frame not in self.plot_frame_step_id_polygon:
                            self.plot_frame_step_id_polygon[self.frame] = {}
                        if i not in self.plot_frame_step_id_polygon[self.frame]:
                            self.plot_frame_step_id_polygon[self.frame][i] = {}
                        self.plot_frame_step_id_polygon[self.frame][i][id] = plot
        else:
            self.update_text()
            for plt in self.plot_list:
                plt.clear()
                plt.obs_list.clear()
            for i in range(min(len(self.plot_list), self.total_scene_num)):
                plt = self.plot_list[i]
                for id, id_data in self.data[self.frame][i].items():
                    color = id_to_rgb(id)
                    traj = id_data['traj']
                    step = min(self.step, len(traj) - 1)
                    print("id: ", id, " i: ", i)
                    print("traj0: ", traj[0])
                    x0, y0, theta0 = traj[0][1:]
                    x, y, theta = traj[step][1:]
                    polygon_x, polygon_y = self.draw_polygon(x, y, theta, x0, y0, theta0, id_data['polygon'])
                    plot = plt.plot(polygon_x, polygon_y, pen=mkPen(color=color, width=3), name=str(id))
                    # if len(traj)>1:
                    #     plot = plt.plot(polygon_x, polygon_y, pen=mkPen(color=color, width=3), name=str(id))
                    # else:
                    #     plot = plt.plot(polygon_x, polygon_y, pen=mkPen(color=color, width=3))

                    if self.frame not in self.plot_frame_step_id_polygon:
                        self.plot_frame_step_id_polygon[self.frame] = {}
                    if i not in self.plot_frame_step_id_polygon[self.frame]:
                        self.plot_frame_step_id_polygon[self.frame][i] = {}
                    self.plot_frame_step_id_polygon[self.frame][i][id] = plot

                    refpath_x = [pt[0] for pt in id_data['refpath']]
                    refpath_y = [pt[1] for pt in id_data['refpath']]
                    plt.plot(refpath_x, refpath_y, pen=None, symbol='o', symbolBrush=mkColor(color), symbolSize=4)
                    plt.obs_list[id] = (x, y)
        
        print ("update vis", time.perf_counter() - start)
        self.update_flag = False
    
    def reset(self):
        self.delete_all_windows()
        self.refresh_windows()
        self.plot_frame_step_id_polygon = {}

    def refresh_windows(self):
        self.scene_blank_list.clear()
        self.plot_list.clear()
        self.scroll_list.clear()

        size = (self.sub_window_size, self.sub_window_size)
        self.resize(self.scene_num * (size[0]+35), self.desktop_height)

        scroll = QScrollArea()
        scroll.setFixedHeight(size[1]+20)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_list.append(scroll)
        scroll_widget = QWidget()
        scroll_widget.setFixedHeight(size[1])
        row = QHBoxLayout()
        row.setSpacing(30)
        row.addStretch()
        for i in range(self.scene_num):
            plot = MousePlotWidget(background='w')
            plot.showGrid(x=True, y=True)
            plot.setFixedSize(size[0], size[1])
            plot.setAspectLocked()
            plot.setTitle("")
            if OPEN_LEGEND: plot.addLegend() # offset=(750, 10) self.
            self.__set_relationship(plot, row, row.addWidget)
            self.plot_list.append(plot)
        row.addStretch()

        self.__set_relationship(row, scroll_widget, scroll_widget.setLayout)
        self.__set_relationship(scroll_widget, scroll, scroll.setwidget)
        self.__set_relationship(scroll, self.layout, self.layout.addWidget)

        row = QHBoxLayout() # 初始化一行
        self.__set_relationship(row, self.layout, self.layout.addLayout)

        self.__init_operation_buttons()

        self.scroll_bar = QScrollBar(Qt.Horizontal, self)
        self.scroll_bar.valueChanged.connect(self.sync_scroll)
        self.scroll_bar.setFixedHeight(30)
        self.__set_relationship(self.scroll_bar, self.layout, self.layout.addWidget)

        scroll = QScrollArea()
        scroll.setFixedHeight(size[1]+20)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.scroll_list.append(scroll)
        scroll_widget = QWidget()
        scroll_widget.setFixedHeight(size[1])
        row = QHBoxLayout()
        row.setSpacing(30)
        row.addStretch()
        box_size = (80, 30)
        for i in range(self.scene_num):
            scene_layout = QVBoxLayout() # 从上到下竖直布局
            scene_blank = self.__add_text(size, self.temp, scene_layout)
            scene_blank.verticalScrollBar().valueChanged.connect(self.text_sync_scroll)
            self.__set_relationship(scene_layout, row, row.addLayout)
            self.scene_blank_list.append(scene_blank)
        row.addStretch()

        self.__set_relationship(row, scroll_widget, scroll_widget.setLayout)
        self.__set_relationship(scroll_widget, scroll, scroll.setWidget)
        self.__set_relationship(scroll, self.layout, self.layout.addWidget)

    def __set_relationship(self, child, father, father_function):
        father_function(child)
        self.delete_dict.append((child, father))

    def text_sync_scroll(self, e):
        for text_scroll in self.scene_blank_list:
            text_scroll.verticalScrollBar().setValue(e)
    
    def sync_scroll (self, e):
        for scroll in self.scroll_list:
            scroll.horizontalScrollBar().setValue(self.scroll_bar.value() / self.scroll_bar.maximum () *\
            scroll.horizontalScrollBar().maximum())
    
    def __add_combobox(self, init, value_list, size, function, father_widget):
        combobox = QComboBox()
        combobox.currentIndexChanged[int].connect(function)
        combobox.addItem(init)
        combobox.addItems([v for v in value_list if v != init])
        self.__set_relationship(combobox, father_widget, father_widget.addwidget)
        return combobox
    
    def __add_button(self, text, size, function, father_widget):
        button = QPushButton()
        button.setText(text) # 按钮名称
        button.setFixedSize(size[0], size[1])
        button.clicked.connect(function) # 按下后触发draw函数
        self.__set_relationship(button, father_widget, father_widget.addWidget)
        return button
    
    def __add_text(self, size, function, father_widget):
        text = QTextEdit(self)
        text.setFixedSize(size[0], size[1])
        text.setFontPointSize(15)
        text.textChanged.connect(function)
        self.__set_relationship(text, father_widget, father_widget.addWidget)
        return text

    def __add_slider(self, min, init, max, size, function, father_widget):
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min) # 设置最小值
        slider.setMaximum(max) # 没置最大値
        slider.setValue(init) # 设置初始值
        slider.setFixedSize(size[0], size[1])
        slider.valueChanged.connect(function) # 监听滑动条值变化事件并打印当前刻度
        self.__set_relationship(slider, father_widget, father_widget.addWidget)
        return slider
    
    def __add_label(self, text, size, father_widget):
        label = QLabel(text, self)
        label.setText(text)
        label.setFixedSize(size[0], size[1])
        self.__set_relationship(label, father_widget, father_widget.addWidget)
        return label
    
    def __init_operation_buttons(self):
        row = QHBoxLayout() # 水平排布
        row.addStretch()

        size = (70, 30)
        slider_size = (300, 30)
        label_size = (110, 30)
        self.__add_button("<<", size, self.frame_decrease, row)
        self.frame_slider = self.__add_slider(1, self.frame, self.total_frame, slider_size, self.frame_slider_changed, row)
        self.__add_button(">>", size, self.frame_increase, row)
        self.jump_to_frame = self.__add_text(size, self.frame_enter, row)
        self.frame_label = self.__add_label(f"frame: {self.frame}/{self.total_frame}", label_size, row)

        self.__add_button("<", size, self.step_decrease, row)
        self.step_slider = self.__add_slider(0, self.step, self.total_step, slider_size, self.step_slider_changed, row)
        self.__add_button(">", size, self.step_increase, row)
        self.jump_to_step = self.__add_text(size, self.step_enter, row)
        self.step_label = self.__add_label(f"step: {self.step}/{self.total_step}", label_size, row)

        self.__add_button("-", size, self.scene_text_decrease, row)
        self.jump_to_scene = self.__add_text(size, self.scene_text_enter, row)
        self.__add_button("+", size, self.scene_text_increase, row)
        self.scene_label = self.__add_label(f"scene: {self.scene_num}/{self.total_scene_num}", label_size, row)

        self.__add_button ("Refresh", size, self.refresh, row)

        row.addStretch()

        self.__set_relationship(row, self.layout, self.layout.addLayout)
    
    def refresh(self):
        if self.frame == self.total_frame and self.step == 0:
            return
        self.frame = self.total_frame
        self.step = 0
        self.update_vis(False)
    
    def frame_slider_changed(self):
        self.frame_changed(self.frame_slider.value())
    
    def frame_decrease(self):
        self.frame_changed(self.frame - 1)
    
    def frame_increase(self):
        self.frame_changed(self.frame + 1)
    
    def frame_enter(self) :
        msg = self.jump_to_frame.toPlainText()
        if '\n' in msg:
            num = int(msg.strip())
            if 0 < num:
                self.frame_changed(num)
            self.jump_to_frame.setText('')
    
    def frame_changed (self, frame):
        self.frame = max(1, min(self.total_frame, frame))
        if frame < 1 or frame > self.total_frame: return
        self.step = 0
        self.update_vis(False)
    
    def step_slider_changed(self):
        self.step_changed(self.step_slider.value())
    
    def step_decrease(self):
        self.step_changed(self.step - 1)
    
    def step_increase(self):
        self.step_changed(self.step + 1)
    
    def step_enter(self):
        msg = self.jump_to_step.toPlainText()
        if '\n' in msg:
            num = int(msg.strip())
            if 0 <= num:
                self.step_changed(num)
            self.jump_to_step.setText('')
    
    def step_changed(self, step):
        self.step = max(0, min(self.total_step, step))
        if step < 0 or step > self.total_step: return
        self.update_vis(True)
    
    def scene_text_enter(self):
        msg = self.jump_to_scene.toPlainlext()
        if '\n' in msg:
            num = int(msg.strip())
            if 0 < num:
                self.scene_text_changed(num)
    
    def scene_text_increase(self):
        self.scene_text_changed(self.scene_num + 1)

    def scene_text_decrease(self):
        self.scene_text_changed(self.scene_num - 1)

    def scene_text_changed(self, num):
        num = max(num , 1)
        if num != self.scene_num:
            self.scene_num = num
            self.reset()
            self.update_vis()
    
    def temp(self) :
        pass

    def delete_all_windows(self):
        for child, father in reversed(self.delete_dict):
            # try:
            #     father.removeWidget(child)
            # except:
            #     pass
            # child.setParent(None)
            try:
                child.deleteLater()
            except:
                pass

if __name__ == "__main__":
    # QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    print("11111")
    QCoreApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    app = QApplication(sys.argv)
    desktop = app.desktop()
    myui = MyUI(desktop.width(), desktop.height())
    myui.show()

    app.exec_()