import sys
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *

import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib.transforms as mtransforms

from type import *
import math

matplotlib.use("Qt5Agg")	# 使用 Qt5 作为后端进行画图

class CreateCanvas(FigureCanvasQTAgg):
    """
    新定义一个 FigureCanvasQTAgg 的子类，在子类中创建 figure、ax以及cax
    """
    def __init__(self):
        self.axes = None
        # self.fig = Figure(figsize, dpi)
        self.fig = Figure()
        super(CreateCanvas, self).__init__(self.fig)
        self.create_axes()

    def create_axes(self):
        # 创建子图
        self.ax1 = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)

class MyUI(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        # 数据成员
        self.figure = CreateCanvas()
        self.jump_to = None
        self.scene_blank_1 = None
        self.scene_blank_2 = None
        self.step = 0
        self.total_frame_num = 0
        self.scenes = []

        # 界面初始化
        self.layout = QVBoxLayout() # 从上到下竖直布局
        self.init_ui() # 初始化界面
        
    def init_ui(self):
        self.init_vis_windows()
        self.init_operation_buttons()
        self.init_cost_vis()

        self.setLayout(self.layout)

    """
    部署画板
    """
    def init_vis_windows(self):
        row = QHBoxLayout() # 初始化一行
        row.addWidget(self.figure)
        self.layout.addLayout(row) # 将这一行控件添加到layout排布中去 

    """
    部署cost可视化栏
    """
    def init_cost_vis(self):
        row = QHBoxLayout() # 从左到右水平布局

        # label = QLabel(self)
        # label.setFixedSize(120, 30)
        # label.setText("Scene1") # 标签名称
        # row.addWidget(label)

        self.scene_blank_1 = QTextEdit(self)
        self.scene_blank_1.setFixedSize(500, 100)
        row.addWidget(self.scene_blank_1)


        # label = QLabel(self)
        # label.setFixedSize(120, 30)
        # label.setText("Scene2") # 标签名称
        # row.addWidget(label)

        self.scene_blank_2 = QTextEdit(self)
        self.scene_blank_2.setFixedSize(500, 100)
        row.addWidget(self.scene_blank_2)

        self.layout.addLayout(row) # 将这一行控件添加到layout排布中去

    """
    部署控制键
    """
    def init_operation_buttons(self):
        row = QHBoxLayout()  # 水平排布

        button = QPushButton()
        button.setText("Load")  # 按钮名称
        button.setFixedSize(80, 30)
        button.clicked.connect(self.load)  # 按下后触发draw函数
        row.addWidget(button)  # 将该按钮添加到该行(row)

        button = QPushButton()
        button.setText("Forward")  # 按钮名称
        button.setFixedSize(80, 30)
        button.clicked.connect(self.forward)  # 按下后触发draw函数
        row.addWidget(button)  # 将该按钮添加到该行(row)

        button = QPushButton()
        button.setText("Backward")  # 按钮名称
        button.setFixedSize(80, 30)
        button.clicked.connect(self.backward)  # 按下后触发draw函数
        row.addWidget(button)  # 将该按钮添加到该行(row)

        button = QPushButton()
        button.setText("Jump To")  # 按钮名称
        button.setFixedSize(80, 30)
        button.clicked.connect(self.jump)  # 按下后触发draw函数
        row.addWidget(button)  # 将该按钮添加到该行(row)

        self.jump_to = QTextEdit(self)
        self.jump_to.setFixedSize(80, 30)
        row.addWidget(self.jump_to)

        self.layout.addLayout(row) # 将这一行控件添加到layout的竖直排布中去 

    def forward(self):
        if self.step > 0:
            self.step -= 1 
        self.update_vis()
        print("step: ", self.step)

    def backward(self):
        if self.step < self.total_frame_num - 1:
            self.step += 1
        self.update_vis()
        print("step: ", self.step)

    def jump(self):
        step_txt = self.jump_to.toPlainText()
        try:
            self.step = max(0, min(self.total_frame_num - 1, int(step_txt)))
        except:
            print("can not transfer text to int")
        self.update_vis()
        print("jump to: ", self.step)


    """
    从每一帧dump的数据中加载可视化内容
    """
    def load(self):
        # TODO: load context from txt
        self.total_frame_num = 0
        scene_content = SceneContent(RefPath())
        scene_content.ego_path.label = "O"
        scene_content.ego_path.pts = [[0., 0., 0.], [10., 20., 0.0], [20., 30., 0.0], [30., 35., 0.0], [40., 37.5, 0.0]]
        scene_content.states_dict["0"] = [[0., 0., 0.], [10., 20., 0.0], [20., 30., 0.0]]
        scene_content.max_frame = len(scene_content.states_dict["0"])
        scene_content.costs_dict["0"] = {"safety": 1.034, "effiency": 3.127}
        self.total_frame_num = max(self.total_frame_num, scene_content.max_frame)
        self.scenes.append(scene_content)
        self.scene_blank_1.setPlainText(scene_content.text())


        scene_content = SceneContent(RefPath())
        scene_content.ego_path.label = "K"
        scene_content.ego_path.pts = [[0., 0., 0.], [10.0, 40.0, 0.0], [20., 30., 0.0], [30., 25., 0.0], [40., 20., 0.0]]
        scene_content.states_dict["0"] = [[0., 0., 0.], [10., 40., 0.0], [20., 30., 0.0]]
        scene_content.max_frame = len(scene_content.states_dict["0"])
        scene_content.costs_dict["0"] = {"safety": 2.194, "effiency": 5.397}
        self.total_frame_num = max(self.total_frame_num, scene_content.max_frame)
        self.scenes.append(scene_content)
        self.scene_blank_2.setPlainText(scene_content.text())

        self.update_vis()

    """
    更新画板 
    """
    def update_vis(self):
        # TODO: draw state and refpath
        if len(self.scenes) > 0:
            self.draw(self.figure.ax1, self.scenes[0])
        if len(self.scenes) > 1:
            self.draw(self.figure.ax2, self.scenes[1])
        self.figure.draw()
        print("draw!!")
    
    def draw(self, ax, scene):
        ax.cla()
        # draw refpath
        x_list = []
        y_list = []
        for pt in scene.ego_path.pts:
            x_list.append(pt[0])
            y_list.append(pt[1])
        ax.plot(x_list, y_list)
        # draw ego state
        if "0" in scene.states_dict and len(scene.states_dict["0"]) > 0:
            step = min(self.step, len(scene.states_dict["0"]) - 1)
            state = scene.states_dict["0"][step]
            param = scene.ego_param
            self.draw_box(ax, state[0], state[1], state[2], param[0], param[1], param[2], param[3])
        # draw obs state
        for id, states in scene.states_dict.items():
            if id == "0":
                continue
            assert(id in scene.param_dict)
            param = scene.param_dict[id]
            self.draw_box(ax, state[0], state[1], state[2], param[0], param[1], 0.5 * param[0], 0.5 * param[0]) 

    def draw_box(self, ax, x, y, theta, length, width, front2vrp, back2vrp):
        half_width = 0.5 * width
        lf_x = x + math.cos(theta) * front2vrp - math.sin(theta) * half_width
        lf_y = y + math.sin(theta) * front2vrp + math.cos(theta) * half_width

        rf_x = x + math.cos(theta) * front2vrp + math.sin(theta) * half_width
        rf_y = y + math.sin(theta) * front2vrp - math.cos(theta) * half_width

        rb_x = x - math.cos(theta) * back2vrp + math.sin(theta) * half_width
        rb_y = y - math.sin(theta) * back2vrp - math.cos(theta) * half_width

        lb_x = x - math.cos(theta) * back2vrp - math.sin(theta) * half_width
        lb_y = y - math.sin(theta) * back2vrp + math.cos(theta) * half_width

        ax.plot([lf_x, rf_x, rb_x, lb_x, lf_x], [lf_y, rf_y, rb_y, lb_y, lf_y])



if __name__ == '__main__':
    app = QApplication( sys.argv )

    myui = MyUI()
    myui.resize( 500, 500 )
    myui.show()

    app.exec_()