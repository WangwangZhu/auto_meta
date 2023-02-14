#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <cstdlib>
#include <QPixmap>
#include <QImage>
Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);




    //1. jvzhen_config.sh里面的内容需要匹配autometa
    //2. 终端的名字匹配代码内容
    //3.

    //1. btn指令链接！
    connect(ui->pushButton1, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-1 ???????'   -- bash -c 'source jvzhen_config.sh;"
                    "ros2 launch visualization vehicle_path_visualization_rviz_launch.py ; $SHELL'");
    });
    connect(ui->pushButton2, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-2 ???????'   -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run integrated_navigation_system ins_d_data_parse ; $SHELL'");
    });

    connect(ui->pushButton3, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-3 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 launch lqr_pid_trajectory_tracking lqr_pid_trajectory_tracking_dynamics_launch.py ; $SHELL'");
    });
    connect(ui->pushButton4, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-4 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run matrix_vehicle_chassis_communication matrix_chassis_rec_send ;$SHELL'");
    });
    connect(ui->pushButton5, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-5 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run sensor_fusion sensor_fusion_node ;$SHELL'");
    });
    connect(ui->pushButton6, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-6 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run fsm_decision_making fsm_decision_making_node ;$SHELL'");
    });
    connect(ui->pushButton7, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-7 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 launch basic_planner basic_planner_launch.py ;$SHELL'");
    });
    connect(ui->pushButton8, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton9, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton10, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton11, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton12, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton13, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton14, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton15, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });
    connect(ui->pushButton16, &QPushButton::clicked,[=](){
        std::system("gnome-terminal --window --title='terminal-8 ???????' -- bash -c 'source jvzhen_config.sh;"
                    "ros2 run cpp_pubsub sub_pub ;$SHELL'");
    });

    //2. 页面修饰

    QString pic_xiaohui(":/picture/xiaohui.png");
    QImage *img = new QImage;
    QImage *rescale_img = new QImage;

    img->load(pic_xiaohui);
    QSize n1 = img->size();
    * rescale_img = img->scaled(int(500/4),int(500/4),Qt::KeepAspectRatio);
    QSize n2 = rescale_img->size();
    qDebug()<<n1<<endl<<n2<<endl;


    ui->xiaohui->resize(rescale_img->width(),rescale_img->height());
    ui->widget1->resize(ui->widget1->width(), rescale_img->height());
    ui->xiaohui->setPixmap(QPixmap::fromImage(*rescale_img));



}
void Widget::paintEvent(QPaintEvent *)
{
    //2. 页面修饰
//    QPixmap *pic1 = new QPixmap(":/picture/xiaohui.png");
//    pic1->scaled(ui->xiaohui->size(),Qt::IgnoreAspectRatio,Qt::SmoothTransformation);
//    ui->xiaohui->setScaledContents(true);
//    ui->xiaohui->setPixmap(*pic1);
}
Widget::~Widget()
{
    delete ui;
}

