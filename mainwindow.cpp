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


    // RRT Algorithm
    // for(int i = 0; i < renderArea->rrt->max_iter; i++) {
    //     Node *q = rrt->getRandomNode();
    //     if (q) {
    //         Node *qNearest = rrt->nearest(q->position);
    //         if (rrt->distance(q->position, qNearest->position) > rrt->step_size) {
    //             Vector2f newConfig = rrt->newConfig(q, qNearest);
    //             if (!rrt->obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
    //                 Node *qNew = new Node;
    //                 qNew->position = newConfig;
    //                 rrt->add(qNearest, qNew);
    //             }
    //         }
    //     }
    //     if (rrt->reached()) {
    //         ui->statusBox->setText(tr("Reached Destination!"));
    //         break;
    //     }
    // }


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
    //RRT* Algorithm
    for(int i = 0; i < renderArea-> rrt->max_iter; i++) {
        Node *q = rrt->getRandomNode();
        if(q){
            Node *qNearest = rrt->nearest(q->position);
            // int counter = 10;
            // while(rrt->distance(q->position, qNearest->position) <= rrt->step_size && counter >= 0){
            //     //repeat for 10 times to search for a valid qNearest
            //     counter -= 1;
            //     qNearest = rrt->nearest(q->position);
            // }
            
            if (rrt->distance(q->position, qNearest->position) > rrt->step_size) {
                Vector2f newConfig = rrt->newConfig(q, qNearest);
                //gives out the new node position in terms of step-size.
                // Node *nearestGroup = rrt->nearestgroup(qNearest->position);
                if (!rrt->obstacles->isSegmentInObstacle(newConfig, qNearest->position)) {
                    Node *qNew = new Node;
                    qNew->id = rrt->nodes.size();
                    qNew->position = newConfig;
                    qNew->parent = NULL;
                    float newcost = rrt->get_cost(qNearest, qNew);
                    qNew->cost = newcost + qNearest->cost;
                    rrt->update(qNew);
                    if(qNew->parent == NULL){
                        rrt->add(qNearest,qNew);
                    }
                }
            }   
        }
        if (rrt->reached()) {
            ui->statusBox->setText(tr("Reached Destination!"));
            break;
        }
    } 
        Node *q;
        if (rrt->reached()) {
            q = rrt->lastNode;
        }
        else
        {
            // if not reached yet, then shortestPath will start from the closest node to end point.
            q = rrt->nearest(rrt->endPos);
            ui->statusBox->setText(tr("Exceeded max iterations!"));
        }
        // generate shortest path to destination.
        while (q != NULL) {
            rrt->path.push_back(q);
            q = q->parent;
        }
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
    printf("where1?\n");
    printf("root:id: %d\n", rrt->root->id);
    rrt->deleteNodes(rrt->root,0);
        printf("where2?\n");
    rrt->nodes.clear();
        printf("where3?\n");
    rrt->nodes.resize(0);
        printf("where4?\n");
    rrt->path.clear();
        printf("where5?\n");
    rrt->path.resize(0);
        printf("where6?\n");
    rrt->initialize();
        printf("where7?\n");
    renderArea->update();
}

MainWindow::~MainWindow()
{
    delete ui;
}
