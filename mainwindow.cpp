#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    renderArea = ui->renderArea;
    rrt = renderArea->rrt;
    simulated = false;
}

// select sub start and goal pairs
void thread_set_args(int ThreadID, int MaxIter, int StepSize, ThreadArgs *ThreadArg, RenderArea *rendertool, Obstacles *obs_pointer) {
    // when thread id is 0, start is global start
    if (ThreadID == 0) {
        ThreadArg->start_x = START_POS_X;
        ThreadArg->start_y = START_POS_Y;
    } else {
        ThreadArg->start_x = (START_POS_X + END_POS_X) * ThreadID / (MAX_THREADS * 1.0f);
        ThreadArg->start_y = (START_POS_Y + END_POS_Y) * ThreadID / (MAX_THREADS * 1.0f);
    }
    // when thread id is max_thread-1, goal is global goal
    if (ThreadID == MAX_THREADS - 1) {
        ThreadArg->goal_x = END_POS_X;
        ThreadArg->goal_y = END_POS_Y;
    } else {
        ThreadArg->goal_x = (START_POS_X + END_POS_X) * (ThreadID + 1) / (MAX_THREADS * 1.0f);
        ThreadArg->goal_y = (START_POS_Y + END_POS_Y) * (ThreadID + 1) / (MAX_THREADS * 1.0f);
    }
    ThreadArg->max_iterations = (int) MaxIter / MAX_THREADS;
    ThreadArg->step_size = StepSize;
    ThreadArg->path_found = false;
    ThreadArg->renderer = rendertool;
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
    RenderArea *renderTool = ThreadArg->renderer;
    for(int i = 0; i < ThreadArg->max_iterations; i++) {
        Node *q = thread_rrt.getRandomNode();
        if (q) {
            Node *qNearest = thread_rrt.nearest(q->position);
            if (thread_rrt.distance(q->position, qNearest->position) > thread_rrt.step_size) {
                Vector2f newConfig = thread_rrt.newConfig(q, qNearest);
                if (!thread_rrt.obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
                    Node *qNew = new Node;
                    qNew->position = newConfig;
                    thread_rrt.add(qNearest, qNew);
                }
            }
        }
        if (thread_rrt.reached()) {
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

    assert(rrt->step_size > 0);
    assert(rrt->max_iter > 0);

    // define threads
    pthread_t threads[MAX_THREADS];
    ThreadArgs args[MAX_THREADS];

    // use pthreads to create substart and subgoal pairs
    for (int i = 0; i < MAX_THREADS; i++) {
        thread_set_args(i, rrt->max_iter, rrt->step_size, &args[i], renderArea, rrt->obstacles);
    }

    // launch threads
    for (int i = 1; i < MAX_THREADS; i++) {
        pthread_create(&threads[i], NULL, thread_start, &args[i]);
    }

    thread_start(&args[0]);

    // wait for threads to finish
    for (int i = 1; i < MAX_THREADS; i++) {
        pthread_join(threads[i], NULL);
    }

    // check if all threads have reached their destinations
    int success = 1;
    for (int i = 0; i < MAX_THREADS; i++) {
        if (args[i].path_found == false) {
            success = 0;
            break;
        } else {
            rrt->nodes.insert(rrt->nodes.end(), args[i].nodes.begin(), args[i].nodes.end());
        }
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
