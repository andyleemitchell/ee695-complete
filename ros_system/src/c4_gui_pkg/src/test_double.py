#!/usr/bin/env python3

import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QGroupBox, QRadioButton, QPushButton, QLabel,
                             QSizePolicy, QGridLayout)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QMetaObject, Q_ARG


class GameSettingsGUI(QWidget):
    startGameSignal = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Game Settings')
        self.setGeometry(300, 300, 350, 450)

        main_layout = QVBoxLayout(self)

        self.group_game_type = QGroupBox("Game Type")
        game_type_layout = QVBoxLayout()
        self.rb_human_vs_robot = QRadioButton("Human vs Robot")
        self.rb_robot_vs_robot = QRadioButton("Robot vs Robot")
        self.rb_human_vs_robot.setChecked(True)
        game_type_layout.addWidget(self.rb_human_vs_robot)
        game_type_layout.addWidget(self.rb_robot_vs_robot)
        self.group_game_type.setLayout(game_type_layout)
        main_layout.addWidget(self.group_game_type)

        self.group_counter_colour = QGroupBox("Counter Colour")
        counter_colour_layout = QVBoxLayout()
        self.rb_human_red = QRadioButton("Human Red / Robot Yellow")
        self.rb_human_yellow = QRadioButton("Human Yellow / Robot Red")
        self.rb_human_red.setChecked(True)
        counter_colour_layout.addWidget(self.rb_human_red)
        counter_colour_layout.addWidget(self.rb_human_yellow)
        self.group_counter_colour.setLayout(counter_colour_layout)
        main_layout.addWidget(self.group_counter_colour)

        self.group_difficulty = QGroupBox("Robot Difficulty")
        difficulty_layout = QVBoxLayout()
        self.rb_easy = QRadioButton("Easy")
        self.rb_hard = QRadioButton("Hard")
        self.rb_impossible = QRadioButton("Impossible")
        self.rb_easy.setChecked(True)
        difficulty_layout.addWidget(self.rb_easy)
        difficulty_layout.addWidget(self.rb_hard)
        difficulty_layout.addWidget(self.rb_impossible)
        self.group_difficulty.setLayout(difficulty_layout)
        main_layout.addWidget(self.group_difficulty)

        self.group_first_move = QGroupBox("First to Move")
        first_move_layout = QVBoxLayout()
        self.rb_first_human = QRadioButton("Human")
        self.rb_first_robot = QRadioButton("Robot")
        self.rb_first_human.setChecked(True)
        first_move_layout.addWidget(self.rb_first_human)
        first_move_layout.addWidget(self.rb_first_robot)
        self.group_first_move.setLayout(first_move_layout)
        main_layout.addWidget(self.group_first_move)

        main_layout.addStretch(1)

        button_layout = QHBoxLayout()
        self.btn_start = QPushButton("Start")
        self.btn_finish = QPushButton("Finish")
        button_layout.addStretch(1)
        button_layout.addWidget(self.btn_start)
        button_layout.addWidget(self.btn_finish)
        main_layout.addLayout(button_layout)

        self.rb_human_vs_robot.toggled.connect(self.update_options_state)
        self.rb_robot_vs_robot.toggled.connect(self.update_options_state)
        self.btn_start.clicked.connect(self.on_start_clicked)
        self.btn_finish.clicked.connect(self.on_finish_clicked)

        self.update_options_state()

    @pyqtSlot()
    def update_options_state(self):
        is_human_vs_robot = self.rb_human_vs_robot.isChecked()
        is_robot_vs_robot = self.rb_robot_vs_robot.isChecked()

        self.group_counter_colour.setEnabled(is_human_vs_robot)
        self.group_difficulty.setEnabled(is_human_vs_robot or is_robot_vs_robot)
        self.group_first_move.setEnabled(is_human_vs_robot)
        self.rb_first_human.setEnabled(is_human_vs_robot)

    @pyqtSlot()
    def on_start_clicked(self):
        config = {}
        # get selected options
        if self.rb_human_vs_robot.isChecked():
            config['game_type'] = 'Human vs Robot'
        else:
            config['game_type'] = 'Robot vs Robot'

        if self.group_counter_colour.isEnabled():
            if self.rb_human_red.isChecked():
                config['human_colour'], config['robot_colour'] = 'Red', 'Yellow'
            else:
                config['human_colour'], config['robot_colour'] = 'Yellow', 'Red'
        else:
            config['human_colour'], config['robot_colour'] = None, None

        if self.group_difficulty.isEnabled():
            if self.rb_easy.isChecked(): config['difficulty'] = 'Easy'
            elif self.rb_hard.isChecked(): config['difficulty'] = 'Hard'
            else: config['difficulty'] = 'Impossible'
        else:
            config['difficulty'] = 'Easy'

        if self.group_first_move.isEnabled():
            if self.rb_first_human.isChecked(): config['first_move'] = 'Human'
            else: config['first_move'] = 'Robot'
        else:
            config['first_move'] = 'Robot'

        rospy.loginfo("Start button clicked. Configuration collected.")
        print(f"Settings Config: {config}")

        self.startGameSignal.emit(config)
        self.close()

    @pyqtSlot()
    def on_finish_clicked(self):
        rospy.loginfo("Finish button clicked. Shutting down application.")
        QApplication.instance().quit()

    def closeEvent(self, event):
        rospy.loginfo("Settings GUI closed.")
        event.accept()


class GameDisplayGUI(QWidget):
    restartGameSignal = pyqtSignal()
    updateImageSignal = pyqtSignal(int, QPixmap)
    updateDataSignal = pyqtSignal(int, str)

    def __init__(self, game_config):
        super().__init__()
        self.game_config = game_config
        self.bridge = CvBridge()
        self.image_labels = []
        self.data_labels = []
        self.image_subs = []
        self.data_subs = []

        self.last_pixmaps = {}

        self.image_topic_names = [
            "/gui/image_topic/1",
            "/gui/image_topic/2",
            "/gui/image_topic/3",
            "/gui/image_topic/4"
        ]
        self.data_topic_names = [
            "/gui/game_data/1",
            "/gui/game_data/2",
            "/gui/game_data/3",
            "/gui/game_data/4",
            "/gui/game_data/5",
            "/gui/game_data/6"
        ]

        self.init_ui()
        self.connect_ros()
        self.resize(600, 550)

    def init_ui(self):
        self.setWindowTitle('Game Display')

        main_layout = QVBoxLayout(self)
        grid_layout = QGridLayout()

        image_positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for i, pos in enumerate(image_positions):
            img_label = QLabel(f"Image {i+1}\n(Waiting for topic\n{self.image_topic_names[i]})")
            img_label.setAlignment(Qt.AlignCenter)
            img_label.setFrameShape(QLabel.Box)
            img_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            img_label.setMinimumSize(100, 75)
            grid_layout.addWidget(img_label, pos[0], pos[1])
            self.image_labels.append(img_label)
            self.last_pixmaps[i] = None

        data_layout_left = QVBoxLayout()
        for i in range(3):
            idx_data = i
            data_label = QLabel(f"Data {idx_data+1}: -")
            data_label.setFrameShape(QLabel.StyledPanel)
            data_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            data_layout_left.addWidget(data_label)
            self.data_labels.append(data_label)
        grid_layout.addLayout(data_layout_left, 2, 0)

        data_layout_right = QVBoxLayout()
        for i in range(3, 6):
            idx_data = i
            data_label = QLabel(f"Data {idx_data+1}: -")
            data_label.setFrameShape(QLabel.StyledPanel)
            data_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            data_layout_right.addWidget(data_label)
            self.data_labels.append(data_label)
        grid_layout.addLayout(data_layout_right, 2, 1)

        grid_layout.setRowStretch(0, 3)
        grid_layout.setRowStretch(1, 3)
        grid_layout.setRowStretch(2, 1)
        grid_layout.setColumnStretch(0, 1)
        grid_layout.setColumnStretch(1, 1)

        main_layout.addLayout(grid_layout)

        button_layout = QHBoxLayout()
        self.btn_end_game = QPushButton("End Game")
        self.btn_restart_game = QPushButton("Restart Game")
        button_layout.addStretch(1)
        button_layout.addWidget(self.btn_end_game)
        button_layout.addWidget(self.btn_restart_game)
        main_layout.addLayout(button_layout)

        self.btn_end_game.clicked.connect(self.on_end_game_clicked)
        self.btn_restart_game.clicked.connect(self.on_restart_game_clicked)
        self.updateImageSignal.connect(self.update_image_label)
        self.updateDataSignal.connect(self.update_data_label)

    def connect_ros(self):
        rospy.loginfo("Connecting to ROS topics for Game Display GUI...")
        for i, topic_name in enumerate(self.image_topic_names):
            try:
                callback = lambda msg, index=i: self.image_callback(msg, index)
                sub = rospy.Subscriber(topic_name, Image, callback, queue_size=1)
                self.image_subs.append(sub)
                rospy.loginfo(f"Subscribed to image topic: {topic_name}")
            except Exception as e:
                rospy.logerr(f"Failed to subscribe to {topic_name}: {e}")

        for i, topic_name in enumerate(self.data_topic_names):
            try:
                callback = lambda msg, index=i: self.data_callback(msg, index)
                sub = rospy.Subscriber(topic_name, String, callback, queue_size=10)
                self.data_subs.append(sub)
                rospy.loginfo(f"Subscribed to data topic: {topic_name}")
            except Exception as e:
                rospy.logerr(f"Failed to subscribe to {topic_name}: {e}")

    def image_callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error for image {index} on topic {self.image_topic_names[index]}: {e}")
            return
        except Exception as e:
            rospy.logerr(f"Error processing image {index} on topic {self.image_topic_names[index]}: {e}")
            return

        try:
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)

            self.last_pixmaps[index] = pixmap

            self.updateImageSignal.emit(index, pixmap)
        except Exception as e:
            rospy.logerr(f"Error converting cv_image to QPixmap for index {index}: {e}")


    def data_callback(self, msg, index):
        try:
            self.updateDataSignal.emit(index, msg.data)
        except Exception as e:
             rospy.logerr(f"Error processing data message for index {index}: {e}")

    @pyqtSlot(int, QPixmap)
    def update_image_label(self, index, pixmap):
        if 0 <= index < len(self.image_labels):
            label = self.image_labels[index]
            original_pixmap = self.last_pixmaps.get(index)
            if original_pixmap:
                scaled_pixmap = original_pixmap.scaled(label.size(),
                                                    Qt.KeepAspectRatio,
                                                    Qt.SmoothTransformation)
                label.setPixmap(scaled_pixmap)
        else:
            rospy.logwarn(f"Received image update for invalid index: {index}")

    @pyqtSlot(int, str)
    def update_data_label(self, index, text):
        if 0 <= index < len(self.data_labels):
            self.data_labels[index].setText(f"Data {index+1}: {text}")
        else:
             rospy.logwarn(f"Received data update for invalid index: {index}")

    def resizeEvent(self, event):
        super(GameDisplayGUI, self).resizeEvent(event)

        for index, pixmap in self.last_pixmaps.items():
            if pixmap and 0 <= index < len(self.image_labels):
                label = self.image_labels[index]
                scaled_pixmap = pixmap.scaled(label.size(),
                                              Qt.KeepAspectRatio,
                                              Qt.SmoothTransformation)
                label.setPixmap(scaled_pixmap)

    @pyqtSlot()
    def on_end_game_clicked(self):
        rospy.loginfo("End Game button clicked. Shutting down application.")
        self.cleanup_ros()
        QApplication.instance().quit()

    @pyqtSlot()
    def on_restart_game_clicked(self):
        rospy.loginfo("Restart Game button clicked. Returning to Settings.")
        self.cleanup_ros()
        self.restartGameSignal.emit()
        self.close()

    def cleanup_ros(self):
        rospy.loginfo("Cleaning up ROS subscribers for Game Display GUI.")
        for sub in self.image_subs:
            try:
                sub.unregister()
            except Exception as e:
                rospy.logwarn(f"Error unregistering image sub: {e}")
        for sub in self.data_subs:
            try:
                sub.unregister()
            except Exception as e:
                rospy.logwarn(f"Error unregistering data sub: {e}")
        self.image_subs = []
        self.data_subs = []

    def closeEvent(self, event):
        rospy.loginfo("Game Display GUI closed.")
        self.cleanup_ros()
        event.accept()



class ApplicationController:
    def __init__(self):
        self.settings_gui = None
        self.game_gui = None
        self.app = None

    def show_settings(self):
        rospy.loginfo("Showing Settings GUI")
        if self.game_gui:
            self.game_gui.cleanup_ros() 
            self.game_gui.close()
            self.game_gui = None

        self.settings_gui = GameSettingsGUI()
        self.settings_gui.startGameSignal.connect(self.show_game)
        self.settings_gui.show()

    def show_game(self, config):
        rospy.loginfo("Showing Game Display GUI")
        self.settings_gui = None

        self.game_gui = GameDisplayGUI(config)
        self.game_gui.restartGameSignal.connect(self.show_settings)
        self.game_gui.show()

    def run(self):
        try:
            rospy.init_node('game_application_gui_node', anonymous=True)
            rospy.loginfo("Game Application ROS node started.")
        except rospy.ROSInitException as e:
            print(f"FATAL: Failed to initialize ROS node: {e}")
            sys.exit(1)

        self.app = QApplication(sys.argv)

        self.show_settings()

        status = self.app.exec_()

        rospy.loginfo("Qt application event loop finished.")
        sys.exit(status)


if __name__ == '__main__':
    try:
        controller = ApplicationController()
        controller.run()
    except rospy.ROSInterruptException:
        print("ROS node interrupted. Exiting.")
    except Exception as e:
        print(f"Error : {e}")
        exit()
