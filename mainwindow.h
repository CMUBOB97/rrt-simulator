#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <pthread.h>
#include <QDebug>
#include "renderarea.h"
#include "ctimer.h"
#include "rrt.h"

#define MAX_THREADS 1
#define THRESHOLD 30

typedef struct {
    float start_x, start_y;
    float goal_x, goal_y;
    int max_iterations, step_size;
    bool path_found;
    Obstacles *obstacles;
    vector<Node *> nodes;
} ThreadArgs;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private:
    Ui::MainWindow *ui;
    RenderArea *renderArea;
    RRT *rrt;
    bool simulated;
private slots:
    void on_startButton_clicked();
    void on_resetButton_clicked();
};

#endif // MAINWINDOW_H
