#include "mainwindow.h"

#include <QApplication>


//按键事件
void keyPressEvent(QKeyEvent *event);
void keyReleaseEvent(QKeyEvent *event);

//用户按键控制轴号
int axisNum;
void QKeyBoard::keyPressEvent(QKeyEvent *event){
    switch(event->key()){
        case Qt::Key_Escape:
            this->ui.textEdit_press->append("Key_Escape Press");
            break;
        case Qt::Key_Tab:
            this->ui.textEdit_press->append("Key_Tab Press");
            break;
        case Qt::Key_Enter:
            this->ui.textEdit_press->append("Key_Enter Press");
            break;
        case Qt::Key_Delete:
            this->ui.textEdit_press->append("Key_Delete Press");
            break;
        case Qt::Key_Space:
            this->ui.textEdit_press->append("Key_Space Press");
            break;
        case Qt::Key_Left:
            this->ui.textEdit_press->append("Key_Left Press");
            break;
        case Qt::Key_Up:
            this->ui.textEdit_press->append("Key_Up Press");
            break;
        case Qt::Key_Right:
            this->ui.textEdit_press->append("Key_Right Press");
            break;
        case Qt::Key_Down:
            this->ui.textEdit_press->append("Key_Down Press");
            break;
        /*default:
            this->ui.textEdit->append("KeyEvent");*/
    }
}

void QKeyBoard::keyReleaseEvent(QKeyEvent *event){
    switch(event->key()){
        case Qt::Key_Escape:
            this->ui.textEdit_release->append("Key_Escape Release");
            break;
        case Qt::Key_Tab:
            this->ui.textEdit_release->append("Key_Tab Release");
            break;
        case Qt::Key_Enter:
            this->ui.textEdit_release->append("Key_Enter Release");
            break;
        case Qt::Key_Delete:
            this->ui.textEdit_release->append("Key_Delete Release");
            break;
        case Qt::Key_Space:
            this->ui.textEdit_release->append("Key_Space Release");
            break;
        case Qt::Key_Left:
            this->ui.textEdit_release->append("Key_Left Release");
            break;
        case Qt::Key_Up:
            this->ui.textEdit_release->append("Key_Up Release");
            break;
        case Qt::Key_Right:
            this->ui.textEdit_release->append("Key_Right Release");
            break;
        case Qt::Key_Down:
            this->ui.textEdit_release->append("Key_Down Release");
            break;
            /*default:
            this->ui.textEdit->append("KeyEvent");*/
    }
}

void Robot::keyPressEvent(QKeyEvent *event)
{
    double curValue[6] = {0};
    //获取当前机器人各个轴的转动角度
    getAxis(curValue);
    switch(event->key()){
        case Qt::Key_Equal:
            {
                switch(this->axisNum){
                    case 1:
                        ui.horizontalSlider_Axis1->setValue(curValue[0]*100+1000);
                        break;
                    case 2:
                        ui.horizontalSlider_Axis2->setValue(curValue[1]*100+1000);
                        break;
                    case 3:
                        ui.horizontalSlider_Axis3->setValue(curValue[2]*100+1000);
                        break;
                    case 4:
                        ui.horizontalSlider_Axis4->setValue(curValue[3]*100+1000);
                        break;
                    case 5:
                        ui.horizontalSlider_Axis5->setValue(curValue[4]*100+1000);
                        break;
                    case 6:
                        ui.horizontalSlider_Axis6->setValue(curValue[5]*100+1000);
                        break;
                    default:
                        break;
                }
            break;
            }
        case Qt::Key_Minus:
            {
                switch(this->axisNum){
                    case 1:
                        ui.horizontalSlider_Axis1->setValue(curValue[0]*100-1000);
                        break;
                    case 2:
                        ui.horizontalSlider_Axis2->setValue(curValue[1]*100-1000);
                        break;
                    case 3:
                        ui.horizontalSlider_Axis3->setValue(curValue[2]*100-1000);
                        break;
                    case 4:
                        ui.horizontalSlider_Axis4->setValue(curValue[3]*100-1000);
                        break;
                    case 5:
                        ui.horizontalSlider_Axis5->setValue(curValue[4]*100-1000);
                        break;
                    case 6:
                        ui.horizontalSlider_Axis6->setValue(curValue[5]*100-1000);
                        break;
                    default:
                        break;
                }
                break;
            }
    }
}

void Robot::keyReleaseEvent(QKeyEvent *event)
{
    //在松键的时候记录用户按下的哪个键
    switch(event->key()){
        case Qt::Key_1:
            axisNum = 1;
            break;
        case Qt::Key_2:
            axisNum = 2;
            break;
        case Qt::Key_3:
            axisNum = 3;
            break;
        case Qt::Key_4:
            axisNum = 4;
            break;
        case Qt::Key_5:
            axisNum = 5;
            break;
        case Qt::Key_6:
            axisNum = 6;
            break;
        default:
            break;


    connect(ui.horizontalSlider_Axis1,SIGNAL(valueChanged(int)),this,SLOT(setRobotPose()));
    connect(ui.horizontalSlider_Axis2,SIGNAL(valueChanged(int)),this,SLOT(setRobotPose()));
    connect(ui.horizontalSlider_Axis3,SIGNAL(valueChanged(int)),this,SLOT(setRobotPose()));
    connect(ui.horizontalSlider_Axis4,SIGNAL(valueChanged(int)),this,SLOT(setRobotPose()));
    connect(ui.horizontalSlider_Axis5,SIGNAL(valueChanged(int)),this,SLOT(setRobotPose()));
    connect(ui.horizontalSlider_Axis6,SIGNAL(valueChanged(int)),this,SLOT(setRobotPose()));

    }
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
