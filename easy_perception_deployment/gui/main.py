# Copyright 2020 Advanced Remanufacturing and Technology Centre
# Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import signal
import sys

from PySide2.QtWidgets import QApplication

from windows.Main import MainWindow

signal.signal(signal.SIGINT, signal.SIG_DFL)
myapp = QApplication(sys.argv)


def main():

    window1 = MainWindow()
    window1.show()

    myapp.exec_()
    sys.exit()


if __name__ == '__main__':
    main()
