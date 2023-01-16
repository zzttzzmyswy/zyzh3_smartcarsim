#include <string>
#include <cstring>
#include <iostream>
#include "GL/glut.h"
#include "scs.h"

unsigned char graph[GRAPH_HEIGHT][GRAPH_WIDTH];

void Display() {
  glTranslated(-0.5, -0.5, 0.0);
  glScaled(0.2, 0.2, 0.2);
  glColor3d(1.0, 0.0, 0.0);

  glBegin(GL_LINE_STRIP);
  for (int i = 0; i < 5; i++) {
    glVertex2d((double)i, 1.0);
    glVertex2d((double)i, 0.0);
  }
  glEnd();

  glLoadIdentity();
  glTranslated(0.0, 0.5, 0.0);
  glColor3d(0.0, 1.0, 0.0);
  glRectd(-0.1, -0.1, 0.1, 0.1);
}
//TODO:此处修改核心控制算法
void AI_Camera() {
    int i = GRAPH_HEIGHT / 2, j;
    int left, right, middle, dir, voltage, Threshold = 50;
    double speed;
    int middle1, middle2, middle3;
    int dir1, dir2, dir3;
    const double pidp = 2;
    const double pidd = 0.9;
    static int dirout = 0, dirout_old = 0;
    static double err, err_last = 0;

    /* 获取当前帧的图像 */
    sGetGraph(graph);

    /* 获取左右两个方向的边线检测 */
    for (j = GRAPH_WIDTH / 2; j > 0; j--)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.45))][j] < Threshold) break;
    left = j;
    for (j = GRAPH_WIDTH / 2; j < GRAPH_WIDTH - 1; j++)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.45))][j] < Threshold) break;
    right = j;
    middle1 = (left + right) / 2;
    /* 获取中线向左右两个方向的边线检测 */
    if(abs(dirout_old)<0)
    {
        for (j = GRAPH_WIDTH / 2 - 10 ; j > 0; j--)
            if (graph[(int)((double)GRAPH_HEIGHT * (0.5))][j] < Threshold) break;
        left = j;
        for (j = GRAPH_WIDTH / 2 + 10; j < GRAPH_WIDTH - 1; j++)
            if (graph[(int)((double)GRAPH_HEIGHT * (0.5))][j] < Threshold) break;
        right = j;
        middle2 = (left + right) / 2;
    }
    else
    {
        for (j = GRAPH_WIDTH / 2 + err_last; j > 0; j--)
            if (graph[(int)((double)GRAPH_HEIGHT * (0.5))][j] < Threshold) break;
        left = j;
        for (j = GRAPH_WIDTH / 2 + err_last; j < GRAPH_WIDTH - 1; j++)
            if (graph[(int)((double)GRAPH_HEIGHT * (0.5))][j] < Threshold) break;
        right = j;
        middle2 = (left + right) / 2;
    }
    /* 获取左右两个方向的边线检测 */
    for (j = GRAPH_WIDTH / 2; j > 0; j--)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.7))][j] < Threshold) break;
    left = j;
    for (j = GRAPH_WIDTH / 2; j < GRAPH_WIDTH - 1; j++)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.7))][j] < Threshold) break;
    right = j;
    middle3 = (left + right) / 2;

    /* 获得距离中线偏差 */
    dir1 = (middle1 - GRAPH_WIDTH / 2);
    dir2 = (middle2 - GRAPH_WIDTH / 2);
    dir3 = (middle3 - GRAPH_WIDTH / 2);

    err = (dir1*1 + dir2*8 + dir3*1)/10;
    err = dir2;
    std::cout << err << std::endl;
    /* PD控制器 */
    dirout = err * pidp + (err - err_last) * pidd;
    err_last = err;
    dirout = dirout * 0.9 + dirout_old * 0.1;
    dirout_old = dirout;

    /* 对输出进行范围限制 */
    dirout = dirout > 100 ? 100 : (dirout < -100 ? -100 : dirout);
    /* 设置舵机角度，正负100度 */
    sSetServoDir(dirout);

    /* 获取平均速度，且单位为角度/s */
    speed = sGetSpeed();
    voltage = (int)((1.5 - speed) * 10.0 + 5.0);
    /* 单位为与电压正相关的值 */
    sSetMotor(voltage);
}

void AI_Electromagnetic() {
  int dir, voltage;
  double speed;
  double left, right;

  sVector pos(-0.1, 0.25, 0.05);
  left = sGetMagnetic(pos).GetX();

  pos.set(0.1, 0.25, 0.05);
  right = sGetMagnetic(pos).GetX();

  dir = (int)(right - left) * 5;
  sSetServoDir(dir);

  speed = sGetSpeed();
  voltage = (int)((1.5 - speed) * 10.0 + 5.0);
  sSetMotor(voltage);
}

unsigned char line[GRAPH_WIDTH];

void AI_Balance() {
  static double Angle = 0.0;
  static double s = 0.0;
  int i, left, right, middle, dir, Threshold = 100;
  double voltage, speed;
  sVector AngularSpeed = sGetAngularSpeed();
  Angle += AngularSpeed.GetX();

  speed = sGetSpeed();
  s += speed;
  s -= 0.3;
  voltage =
      Angle * 10.0 + AngularSpeed.GetX() * 10.0 + s * -20.0 + speed * -10.0;

  sGetLine(line);
  for (i = GRAPH_WIDTH / 2; i > 0; i--)
    if (line[i] < Threshold) break;
  left = i;

  for (i = GRAPH_WIDTH / 2; i < GRAPH_WIDTH - 1; i++)
    if (line[i] < Threshold) break;
  right = i;
  middle = (left + right) / 2;

  dir = (middle - GRAPH_WIDTH / 2) / 10;

  sSetMotorL(+dir - (int)voltage);
  sSetMotorR(-dir - (int)voltage);
}

int main(int argc, char *argv[]) {
  // printf("\n\nargc=%d, argv[0]=%s\n\n", argc, argv[0]);
  int type = 0;
  std::string track = "final8.trk";
  if (argc > 1) {
    type = atoi(argv[1]);
    if (argc > 2) {
      track = argv[2];
    }
  }

  switch (type) {
    default:
    case 0:
      sSetCar(camera);
      sSetAiFunc(AI_Camera);
      sEnableCustomWindow();
      sSetDisplayFunc(Display);
      break;
    case 1:
      sSetCar(balance);
      sSetAiFunc(AI_Balance);
      sSetDepressionAngle(60.0);
      break;
    case 2:
      sSetCar(electromagnetic);
      sSetAiFunc(AI_Electromagnetic);
      break;
  }
  //TODO:修改路径
  sSetTrack((std::string("G:\\专业综合实践-课设资料-2019级\\kf\\ss\\专业综合实践-课设资料-2019级\\项目3资料\\smartcarsim\\build\\VS2010\\Release\\track\\") + std::string(track)).c_str());
  sRegister("I've read the license. And I accept it.");
  scsMainLoop(&argc, argv);

  return 0;
}
