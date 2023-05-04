#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <stdio.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    renderArea = ui->renderArea;
    rrt = renderArea->rrt;
    simulated = false;
}
/*
 * fill the vector with subgoal points
 * based on the connection between 2 distinct obstacles
 * input: obstacle vector containing all obstacles
 * output: a list of line segments
 */
void obstacle_subgoal_points(Obstacles *obs_pointer, std::vector<Vector2f> &subgoals) {

    for (int i = 0; i < obs_pointer->obstacles.size(); i++) {
        for (int j = i + 1; j < obs_pointer->obstacles.size(); j++) {
            // pad the vertices
            Vector2f p_i_1(obs_pointer->obstacles[i].first.x() - 0.5, obs_pointer->obstacles[i].first.y() - 0.5);
            Vector2f p_i_2(obs_pointer->obstacles[i].first.x() - 0.5, obs_pointer->obstacles[i].second.y() + 0.5);
            Vector2f p_i_3(obs_pointer->obstacles[i].second.x() + 0.5, obs_pointer->obstacles[i].first.y() - 0.5);
            Vector2f p_i_4(obs_pointer->obstacles[i].second.x() + 0.5, obs_pointer->obstacles[i].second.y() + 0.5);
            Vector2f p_j_1(obs_pointer->obstacles[j].first.x() - 0.5, obs_pointer->obstacles[j].first.y() - 0.5);
            Vector2f p_j_2(obs_pointer->obstacles[j].first.x() - 0.5, obs_pointer->obstacles[j].second.y() + 0.5);
            Vector2f p_j_3(obs_pointer->obstacles[j].second.x() + 0.5, obs_pointer->obstacles[j].first.y() - 0.5);
            Vector2f p_j_4(obs_pointer->obstacles[j].second.x() + 0.5, obs_pointer->obstacles[j].second.y() + 0.5);
            if(!obs_pointer->isSegmentInObstacle(p_i_1, p_j_1)) {
                Vector2f mid1((p_i_1.x() + p_j_1.x()) / 2, (p_i_1.y() + p_j_1.y()) / 2);
                subgoals.push_back(mid1);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_1, p_j_2)) {
                Vector2f mid2((p_i_1.x() + p_j_2.x()) / 2, (p_i_1.y() + p_j_2.y()) / 2);
                subgoals.push_back(mid2);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_1, p_j_3)) {
                Vector2f mid3((p_i_1.x() + p_j_3.x()) / 2, (p_i_1.y() + p_j_3.y()) / 2);
                subgoals.push_back(mid3);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_1, p_j_4)) {
                Vector2f mid4((p_i_1.x() + p_j_4.x()) / 2, (p_i_1.y() + p_j_4.y()) / 2);
                subgoals.push_back(mid4);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_2, p_j_1)) {
                Vector2f mid5((p_i_2.x() + p_j_1.x()) / 2, (p_i_2.y() + p_j_1.y()) / 2);
                subgoals.push_back(mid5);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_2, p_j_2)) {
                Vector2f mid6((p_i_2.x() + p_j_2.x()) / 2, (p_i_2.y() + p_j_2.y()) / 2);
                subgoals.push_back(mid6);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_2, p_j_3)) {
                Vector2f mid7((p_i_2.x() + p_j_3.x()) / 2, (p_i_2.y() + p_j_3.y()) / 2);
                subgoals.push_back(mid7);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_2, p_j_4)) {
                Vector2f mid8((p_i_2.x() + p_j_4.x()) / 2, (p_i_2.y() + p_j_4.y()) / 2);
                subgoals.push_back(mid8);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_3, p_j_1)) {
                Vector2f mid9((p_i_3.x() + p_j_1.x()) / 2, (p_i_3.y() + p_j_1.y()) / 2);
                subgoals.push_back(mid9);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_3, p_j_2)) {
                Vector2f mid10((p_i_3.x() + p_j_2.x()) / 2, (p_i_3.y() + p_j_2.y()) / 2);
                subgoals.push_back(mid10);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_3, p_j_3)) {
                Vector2f mid11((p_i_3.x() + p_j_3.x()) / 2, (p_i_3.y() + p_j_3.y()) / 2);
                subgoals.push_back(mid11);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_3, p_j_4)) {
                Vector2f mid12((p_i_3.x() + p_j_4.x()) / 2, (p_i_3.y() + p_j_4.y()) / 2);
                subgoals.push_back(mid12);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_4, p_j_1)) {
                Vector2f mid13((p_i_4.x() + p_j_1.x()) / 2, (p_i_4.y() + p_j_1.y()) / 2);
                subgoals.push_back(mid13);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_4, p_j_2)) {
                Vector2f mid14((p_i_4.x() + p_j_2.x()) / 2, (p_i_4.y() + p_j_2.y()) / 2);
                subgoals.push_back(mid14);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_4, p_j_3)) {
                Vector2f mid15((p_i_4.x() + p_j_3.x()) / 2, (p_i_4.y() + p_j_3.y()) / 2);
                subgoals.push_back(mid15);
            }
            if(!obs_pointer->isSegmentInObstacle(p_i_4, p_j_4)) {
                Vector2f mid16((p_i_4.x() + p_j_4.x()) / 2, (p_i_4.y() + p_j_4.y()) / 2);
                subgoals.push_back(mid16);
            }
            
        }
    }

}

// select sub start and goal pairs
void thread_set_args(int ThreadID, int MaxIter, int StepSize, ThreadArgs *ThreadArg, Obstacles *obs_pointer, std::vector<Vector2f> &subgoals) {
    // when thread id is 0, start is global start
    if (ThreadID == 0) {
        ThreadArg->start_x = START_POS_X;
        ThreadArg->start_y = START_POS_Y;
    } else {
        if (subgoals.size() == 0) {
            qDebug() << "Something is wrong with my subgoal calculation";
            ThreadArg->start_x = (START_POS_X + END_POS_X) * ThreadID / (MAX_THREADS * 1.0f);
            ThreadArg->start_y = (START_POS_Y + END_POS_Y) * ThreadID / (MAX_THREADS * 1.0f);
        } else {
            qDebug() << "The subgoal point is: " << subgoals[0].x() << "," << subgoals[0].y();
            ThreadArg->start_x = subgoals[0].x();
            ThreadArg->start_y = subgoals[0].y();
        }
    }
    // when thread id is max_thread-1, goal is global goal
    if (ThreadID == MAX_THREADS - 1) {
        ThreadArg->goal_x = END_POS_X;
        ThreadArg->goal_y = END_POS_Y;
    } else {
        if (subgoals.size() == 0) {
            qDebug() << "Something is wrong with my subgoal calculation";
            ThreadArg->goal_x = (START_POS_X + END_POS_X) * (ThreadID + 1) / (MAX_THREADS * 1.0f);
            ThreadArg->goal_y = (START_POS_Y + END_POS_Y) * (ThreadID + 1) / (MAX_THREADS * 1.0f);
        } else {
            ThreadArg->goal_x = subgoals[0].x();
            ThreadArg->goal_y = subgoals[0].y();
        }
    }
    ThreadArg->max_iterations = (int) MaxIter / MAX_THREADS;
    ThreadArg->step_size = StepSize;
    ThreadArg->path_found = false;
    ThreadArg->obstacles = obs_pointer;
}

void *thread_start(void *VoidThreadArg) {
    // RRT Algorithm
    // first, each thread creates its own subtree
    ThreadArgs *ThreadArg = (ThreadArgs *)VoidThreadArg;
    RRT thread_rrt = RRT();
    thread_rrt.modify_start_goal(ThreadArg->start_x, ThreadArg->start_y,
                                 ThreadArg->goal_x, ThreadArg->goal_y, ThreadArg->obstacles);
    thread_rrt.initialize();
    thread_rrt.setMaxIterations(ThreadArg->max_iterations);
    thread_rrt.setStepSize(ThreadArg->step_size);
    for(int i = 0; i < ThreadArg->max_iterations;) {
        Node *q = thread_rrt.getRandomNode();
        if (q) {
            Node *qNearest = thread_rrt.nearest(q->position);
            if (thread_rrt.distance(q->position, qNearest->position) > thread_rrt.step_size) {
                Vector2f newConfig = thread_rrt.newConfig(q, qNearest);
                if (!thread_rrt.obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    thread_rrt.add(qNearest, qNew);
                    i++;
                }
            }
        }
        if (thread_rrt.reached()) {
            ThreadArg->path_found = true;
            break;
        }
    }
    Node *q;
    if (thread_rrt.reached()) {
        q = thread_rrt.lastNode;
    }
    else
    {
        // if not reached yet, then shortestPath will start from the closest node to end point.
        q = thread_rrt.nearest(thread_rrt.endPos);
    }
    // generate shortest path to destination.
    while (q != NULL) {
        thread_rrt.path.push_back(q);
        q = q->parent;
    }
    // append nodes to args to get things back
    ThreadArg->nodes = thread_rrt.nodes;

    return NULL;
}

/**
 * @brief Start the simulator.
 */
void MainWindow::on_startButton_clicked()
{
    if (simulated) {
        ui->statusBox->setText(tr("Please reset!"));
        renderArea->update();
        return;
    }
    simulated = true;
    // get step size and max iterations from GUI.
    rrt->setMaxIterations(ui->maxIterations->text().toInt());
    rrt->setStepSize(ui->stepSize->text().toInt());

    // vector of potential subgoal points
    std::vector<Vector2f> subgoals;
    obstacle_subgoal_points(rrt->obstacles, subgoals);

    assert(rrt->step_size > 0);
    assert(rrt->max_iter > 0);

    // define threads
    pthread_t threads[MAX_THREADS];
    ThreadArgs args[MAX_THREADS];

    // use pthreads to create substart and subgoal pairs
    for (int i = 0; i < MAX_THREADS; i++) {
        thread_set_args(i, rrt->max_iter, rrt->step_size, &args[i], rrt->obstacles, subgoals);
    }

    // start the timer here
    ctimer_t t;
    ctimer_start(&t);

    // launch threads
    for (int i = 1; i < MAX_THREADS; i++) {
        pthread_create(&threads[i], NULL, thread_start, &args[i]);
    }

    thread_start(&args[0]);

    // wait for threads to finish
    for (int i = 1; i < MAX_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    // stop the timer here
    ctimer_stop(&t);
    ctimer_measure(&t);
    ctimer_print(t, "RRT");

    // check if all threads have reached their destinations
    int success = 1;
    for (int i = 0; i < MAX_THREADS; i++) {
        if (args[i].path_found == false) {
            success = 0;
        }
        rrt->nodes.insert(rrt->nodes.end(), args[i].nodes.begin(), args[i].nodes.end());
        qDebug() << "after thread" << i << "number of nodes in rrt is:" << rrt->nodes.size();
    }
    if (success)
        ui->statusBox->setText(tr("Reached Destination!"));
    else
        ui->statusBox->setText(tr("Exceeded max iterations!"));

    renderArea->update();

}

/**
 * @brief Delete all obstacles, nodes and paths from the simulator.
 */
void MainWindow::on_resetButton_clicked()
{
    simulated = false;
    ui->statusBox->setText(tr(""));
    rrt->obstacles->obstacles.clear();
    rrt->obstacles->obstacles.resize(0);
    rrt->deleteNodes(rrt->root);
    rrt->nodes.clear();
    rrt->nodes.resize(0);
    rrt->path.clear();
    rrt->path.resize(0);
    rrt->initialize();
    renderArea->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
