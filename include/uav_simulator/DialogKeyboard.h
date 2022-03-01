#ifndef DIALOGKEYBOARD_H
#define DIALOGKEYBOARD_H

#include <QDialog>
#include <uav_simulator/droneObjectRos.h>

namespace Ui {
class DialogKeyboard;
}

class DialogKeyboard : public QDialog
{
    Q_OBJECT  
public:
    DroneObjectROS* drone;
public:
    explicit DialogKeyboard(QWidget *parent = 0);
    ~DialogKeyboard();
    void setDrone(DroneObjectROS& drone_){
        this->drone = &drone_;
    }
    void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);
    void testPositionControl();
private:
    Ui::DialogKeyboard *ui;
};

#endif // DIALOGKEYBOARD_H
