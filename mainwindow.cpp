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
            if(p_i_1.x() >= 0 && p_i_1.y() >= 0 && p_j_1.x() >= 0 && p_j_1.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_1, p_j_1)) {
                Vector2f mid1((p_i_1.x() + p_j_1.x()) / 2, (p_i_1.y() + p_j_1.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid1 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid1);
            }
            if(p_i_1.x() >= 0 && p_i_1.y() >= 0 && p_j_2.x() >= 0 && p_j_2.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_1, p_j_2)) {
                Vector2f mid2((p_i_1.x() + p_j_2.x()) / 2, (p_i_1.y() + p_j_2.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid2 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid2);
            }
            if(p_i_1.x() >= 0 && p_i_1.y() >= 0 && p_j_3.x() <= WORLD_WIDTH && p_j_3.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_1, p_j_3)) {
                Vector2f mid3((p_i_1.x() + p_j_3.x()) / 2, (p_i_1.y() + p_j_3.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid3 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid3);
            }
            if(p_i_1.x() >= 0 && p_i_1.y() >= 0 && p_j_4.x() <= WORLD_WIDTH && p_j_4.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_1, p_j_4)) {
                Vector2f mid4((p_i_1.x() + p_j_4.x()) / 2, (p_i_1.y() + p_j_4.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid4 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid4);
            }
            if(p_i_2.x() >= 0 && p_i_2.y() <= WORLD_HEIGHT && p_j_1.x() >= 0 && p_j_1.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_2, p_j_1)) {
                Vector2f mid5((p_i_2.x() + p_j_1.x()) / 2, (p_i_2.y() + p_j_1.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid5 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid5);
            }
            if(p_i_2.x() >= 0 && p_i_2.y() <= WORLD_HEIGHT && p_j_2.x() >= 0 && p_j_2.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_2, p_j_2)) {
                Vector2f mid6((p_i_2.x() + p_j_2.x()) / 2, (p_i_2.y() + p_j_2.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid6 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid6);
            }
            if(p_i_2.x() >= 0 && p_i_2.y() <= WORLD_HEIGHT && p_j_3.x() <= WORLD_WIDTH && p_j_3.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_2, p_j_3)) {
                Vector2f mid7((p_i_2.x() + p_j_3.x()) / 2, (p_i_2.y() + p_j_3.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid7 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid7);
            }
            if(p_i_2.x() >= 0 && p_i_2.y() <= WORLD_HEIGHT && p_j_4.x() <= WORLD_WIDTH && p_j_4.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_2, p_j_4)) {
                Vector2f mid8((p_i_2.x() + p_j_4.x()) / 2, (p_i_2.y() + p_j_4.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid8 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid8);
            }
            if(p_i_3.x() <= WORLD_WIDTH && p_i_3.y() >= 0 && p_j_1.x() >= 0 && p_j_1.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_3, p_j_1)) {
                Vector2f mid9((p_i_3.x() + p_j_1.x()) / 2, (p_i_3.y() + p_j_1.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid9 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid9);
            }
            if(p_i_3.x() <= WORLD_WIDTH && p_i_3.y() >= 0 && p_j_2.x() >= 0 && p_j_2.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_3, p_j_2)) {
                Vector2f mid10((p_i_3.x() + p_j_2.x()) / 2, (p_i_3.y() + p_j_2.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid10 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid10);
            }
            if(p_i_3.x() <= WORLD_WIDTH && p_i_3.y() >= 0 && p_j_3.x() <= WORLD_WIDTH && p_j_3.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_3, p_j_3)) {
                Vector2f mid11((p_i_3.x() + p_j_3.x()) / 2, (p_i_3.y() + p_j_3.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid11 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid11);
            }
            if(p_i_3.x() <= WORLD_WIDTH && p_i_3.y() >= 0 && p_j_4.x() <= WORLD_WIDTH && p_j_4.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_3, p_j_4)) {
                Vector2f mid12((p_i_3.x() + p_j_4.x()) / 2, (p_i_3.y() + p_j_4.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid12 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid12);
            }
            if(p_i_4.x() <= WORLD_WIDTH && p_i_4.y() <= WORLD_HEIGHT && p_j_1.x() >= 0 && p_j_1.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_4, p_j_1)) {
                Vector2f mid13((p_i_4.x() + p_j_1.x()) / 2, (p_i_4.y() + p_j_1.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid13 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid13);
            }
            if(p_i_4.x() <= WORLD_WIDTH && p_i_4.y() <= WORLD_HEIGHT && p_j_2.x() >= 0 && p_j_2.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_4, p_j_2)) {
                Vector2f mid14((p_i_4.x() + p_j_2.x()) / 2, (p_i_4.y() + p_j_2.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid14 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid14);
            }
            if(p_i_4.x() <= WORLD_WIDTH && p_i_4.y() <= WORLD_HEIGHT && p_j_3.x() <= WORLD_WIDTH && p_j_3.y() >= 0 && !obs_pointer->isSegmentInObstacle(p_i_4, p_j_3)) {
                Vector2f mid15((p_i_4.x() + p_j_3.x()) / 2, (p_i_4.y() + p_j_3.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid15 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid15);
            }
            if(p_i_4.x() <= WORLD_WIDTH && p_i_4.y() <= WORLD_HEIGHT && p_j_4.x() <= WORLD_WIDTH && p_j_4.y() <= WORLD_HEIGHT && !obs_pointer->isSegmentInObstacle(p_i_4, p_j_4)) {
                Vector2f mid16((p_i_4.x() + p_j_4.x()) / 2, (p_i_4.y() + p_j_4.y()) / 2);
                bool close = false;
                for (int k = 0; k < subgoals.size(); k++) {
                    Vector2f dist = mid16 - subgoals[k];
                    if (dist.norm() <= THRESHOLD)
                        close = true;
                }
                if(!close)
                    subgoals.push_back(mid16);
            }
            
        }
    }

}

// select sub start and goal pairs
void thread_set_args(int MaxIter, int StepSize, ThreadArgs *ThreadArg, Obstacles *obs_pointer, std::vector<Vector2f> &subgoals) {
    // set a vector for visited subgoals
    std::vector<int> visited(subgoals.size(), 0);

    // single function call for setting all threads
    for (int i = 0; i < MAX_THREADS; i++) {
        // setting start and goal
        if (i == 0) {
            ThreadArg[i].start_x = START_POS_X;
            ThreadArg[i].start_y = START_POS_Y;
        } else {
            ThreadArg[i].start_x = ThreadArg[i-1].goal_x;
            ThreadArg[i].start_y = ThreadArg[i-1].goal_y;
        }

        if (i == MAX_THREADS - 1) {
            ThreadArg[i].goal_x = END_POS_X;
            ThreadArg[i].goal_y = END_POS_Y;
        } else {
            // choose the best subgoal part
            float dist_ratio = (i + 1) * 1.0f / (MAX_THREADS * 1.0f);
            float best_error = -1.f;
            int best_index = 0;
            for (int j = 0; j < subgoals.size(); j++) {
                Vector2f dist_to_start(subgoals[j].x() - START_POS_X, subgoals[j].y() - START_POS_Y);
                Vector2f dist_to_goal(subgoals[j].x() - END_POS_X, subgoals[j].y() - END_POS_Y);
                Vector2f substart(ThreadArg[i].start_x, ThreadArg[i].start_y);
                float current_ratio = dist_to_start.norm() / (dist_to_start.norm() + dist_to_goal.norm());
                if (visited[j] == 0 && (best_error < 0 || abs(current_ratio - dist_ratio) < best_error)) {
                    best_error = abs(current_ratio - dist_ratio);
                    best_index = j;
                }
            }
            // set the index to be visited
            visited[best_index] = 1;
            ThreadArg[i].goal_x = subgoals[best_index].x();
            ThreadArg[i].goal_y = subgoals[best_index].y();
            qDebug() << "The goal for thread " << i << " is " << ThreadArg[i].goal_x << "," << ThreadArg[i].goal_y;
        }
        // set other arguments
        ThreadArg[i].max_iterations = (int) MaxIter / MAX_THREADS;
        ThreadArg[i].step_size = StepSize;
        ThreadArg[i].path_found = false;
        ThreadArg[i].obstacles = obs_pointer;
    }
    
}

void *thread_start(void *VoidThreadArg) {
    // RRT Algorithm
    // first, each thread creates its own subtree
    ThreadArgs *ThreadArg = (ThreadArgs *)VoidThreadArg;
    RRT thread_rrt = RRT();
    ctimer_t update, collision;
    float update_time = 0.0f, collision_time = 0.0f;
    thread_rrt.modify_start_goal(ThreadArg->start_x, ThreadArg->start_y,
                                 ThreadArg->goal_x, ThreadArg->goal_y, ThreadArg->obstacles);
    thread_rrt.initialize();
    thread_rrt.setMaxIterations(ThreadArg->max_iterations);
    thread_rrt.setStepSize(ThreadArg->step_size);
    for(int i = 0; i < ThreadArg->max_iterations;) {
        Node *q = thread_rrt.getRandomNode();
        if (q) {
            ctimer_start(&update);
            Node *qNearest = thread_rrt.nearest(q->position);
            ctimer_stop(&update);
            ctimer_measure(&update);
            update_time += timespec_sec(update.elapsed);
            if (thread_rrt.distance(q->position, qNearest->position) > thread_rrt.step_size) {
                Vector2f newConfig = thread_rrt.newConfig(q, qNearest);
                ctimer_start(&collision);
                if (!thread_rrt.obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
                    ctimer_stop(&collision);
                    ctimer_measure(&collision);
                    collision_time += timespec_sec(collision.elapsed);
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    thread_rrt.add(qNearest, qNew);
                    i++;
                }
            }
        }
        if (thread_rrt.reached()) {
            qDebug() << "tree iteration time: " <<update_time;
            qDebug() << "tree collision time: " <<collision_time;
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
    thread_set_args(rrt->max_iter, rrt->step_size, args, rrt->obstacles, subgoals);

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
