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
//TODO:此处修改核心控制算法 代码编写调试者：张泽韬 ，其余成员：张之焱、戴逸凡
void AI_Camera() {
    int i = GRAPH_HEIGHT / 2, j;
    int left, right, middle, dir, voltage, Threshold = 10;
    double speed;/*速度*/
    int middle1, middle2, middle3;
    static int dir1, dir2, dir3;/*三条线检测到的偏差*/
    const double pidp = 2;/*PID的P参数*/
    const double pidd = 0.9;/*PID的D参数*/
    static int dirout = 0, dirout_old = 0;/*输出记录*/
    static double err, err_last = 0;/*误差记录*/
    static int countL = 0; /*直行道计数*/
    static double mu = 1.5; /*目标速度*/
    static double speed_old, speed_flag = 0, speed_flag1 = 0; /*速度标记*/
    int needadd = 0;/*加速标记*/

    /* 获取当前帧的图像 */
    sGetGraph(graph);

    /* 获取0.5高度左右两个方向的边线检测 */
    /* 将上次误差作为偏差计算基准的因素之一，从而避免转弯边线偏移过大导致边线判断错误 */
    for (j = GRAPH_WIDTH / 2 + err_last; j > 0; j--)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.5))][j] < Threshold) break;
    left = j;
    for (j = GRAPH_WIDTH / 2 + err_last; j < GRAPH_WIDTH - 1; j++)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.5))][j] < Threshold) break;
    right = j;
    middle1 = (left + right) / 2;
    dir1 = (middle1 - GRAPH_WIDTH / 2) + 1;
    /* 获取0.55高度中线向左右两个方向的边线检测 */
    /* 将上次误差作为偏差计算基准的因素之一，从而避免转弯边线偏移过大导致边线判断错误 */
    for (j = GRAPH_WIDTH / 2 + err_last; j > 0; j--)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.55))][j] < Threshold) break;
    left = j;
    for (j = GRAPH_WIDTH / 2 + err_last; j < GRAPH_WIDTH - 1; j++)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.55))][j] < Threshold) break;
    right = j;
    middle2 = (left + right) / 2;
    dir2 = (middle2 - GRAPH_WIDTH / 2) + 1;
    /* 获取0.6高度左右两个方向的边线检测 */
    /* 将上次误差作为偏差计算基准的因素之一，从而避免转弯边线偏移过大导致边线判断错误 */
    for (j = GRAPH_WIDTH / 2 + err_last; j > 0; j--)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.6))][j] < Threshold) break;
    left = j;
    for (j = GRAPH_WIDTH / 2 + err_last; j < GRAPH_WIDTH - 1; j++)
        if (graph[(int)((double)GRAPH_HEIGHT * (0.6))][j] < Threshold) break;
    right = j;
    middle3 = (left + right) / 2;
    dir3 = (middle3 - GRAPH_WIDTH / 2) + 1;
    /*计算误差均数*/
    err = (dir1 * 4 + dir2 * 4 + dir3 * 4) / 12;

    /* PD控制器 */
    dirout = err * pidp + (err - err_last) * pidd;
    err_last = err;
    /*对输出进行滤波*/
    dirout = dirout * 0.9 + dirout_old * 0.1;
    dirout_old = dirout;

    /* 对输出进行范围限制 */
    dirout = dirout > 100 ? 100 : (dirout < -100 ? -100 : dirout);
    /* 设置舵机角度，正负100度 */
    sSetServoDir(dirout);

    /* 获取平均速度，且单位为角度/s */
    speed = sGetSpeed();
    voltage = (int)((mu - speed) * 10.0 + 5.0);
    /* 是否上坡 */
    if (speed < 1.1)
        needadd = 2;
    else
        needadd = 0;
    /* 检测直道 */
    if ((abs(dirout) < 5) || ((abs(dirout) < 12) && speed_flag == 1))
    {
        countL += 1;
        if (countL >= 20)
        {
            needadd = 1;
        }
        if ((speed_flag == 1) && countL >= 10)
            needadd = 1;
    }
    else
    {
        countL = 0;
        needadd = 0;
    }
    /* 当检测到从坡上下来之后，切换速度标记 */
    if ((speed >= 2.7) && (speed_flag1 == 0))
    {
        speed_flag = (speed_flag == 0) ? 1 : 0;
        speed_flag1 = 1;
        //std::cout << "speed_flag" << std::endl;
    }
    else if (speed < 1.3)
    {
        speed_flag1 = 0;
    }
    /* 根据加速标记进行对应的加速方式 */
    if (needadd == 1)
    {
        mu = 5;
        voltage += 5;
    }
    else if (needadd == 2)
    {
        mu = 3;
        voltage += 20;
    }
    else
    {
        if (speed_flag == 1)
            mu = 1.8;
        else
            mu = 1.5;
    }

    speed_old = speed;

    /* 单位为与电压正相关的值 */
    sSetMotor(voltage);
    //std::cout << diravg << " " << dir1 << " " << dir2 << " " << dir3 << " " << err << " " << dirout << std::endl;
    //std::cout << speed << " " << speed_flag << std::endl;
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
      sEnableRoute();
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
