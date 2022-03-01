#include <uav_simulator/DialogKeyboard.h>
#include <QtWidgets>

#include "ui_DialogKeyboard.h"

DialogKeyboard::DialogKeyboard(QWidget *parent)
    : QDialog(parent), ui(new Ui::DialogKeyboard) {
  ui->setupUi(this);
}

DialogKeyboard::~DialogKeyboard() { delete ui; }

void DialogKeyboard::keyPressEvent(QKeyEvent *event) {
  if (!drone)
    return;
  char key = event->key();
  std::cout << "key:" << key << std::endl;
  switch (key) {
  case 'Z':
    // take off
    std::cout << "take off !" << std::endl;
    drone->takeOff();
    break;
  case 'X':
    // land
    drone->land();
    break;
  case 'H':
    drone->hover();
    break;
  case 'W':
    // going up
    drone->rise(0.8f);
    break;
  case 'S':
    // going down
    drone->rise(-0.8f);
    break;
  case 'A':
    // turn left
    drone->yaw(0.8f);
    break;
  case 'D':
    // turn right
    drone->yaw(-0.8f);
    break;
  case 'J':
    // tilt left
    drone->roll(1.5f);
    break;
  case 'L':
    // tilt right
    drone->roll(-1.5f);
    break;
  case 'I':
    // title front
    drone->pitch(1.5f);
    break;
  case 'K':
    // title back
    drone->pitch(-1.5f);
    break;
  case 'T':
    testPositionControl();
    break;

  default:
    drone->hover();
  }
  event->accept();
}

void DialogKeyboard::keyReleaseEvent(QKeyEvent *event) {
  if (!drone)
    return;
  char key = event->key();
  if (!event->isAutoRepeat()) {
    std::cout << "key:" << key << " has been released !" << std::endl;
    if (!drone->isPosctrl)
      drone->hover();
    event->accept();
  } else {
    event->ignore();
  }
}

void DialogKeyboard::testPositionControl() {
  if (drone->isPosctrl) {
    drone->posCtrl(false);
    std::cout << "position control off!" << std::endl;
  } else {
    drone->posCtrl(true);
    std::cout << "Flying to (4.5,-4.5, 6) with position control." << std::endl;
    drone->moveTo(4.5, -4.5, 6);
  }
}