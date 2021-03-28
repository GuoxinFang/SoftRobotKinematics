#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSignalMapper>
#include <QStandardItemModel>
#include <QTimer>

#include "../GLKLib/GLKLib.h"
#include "../QMeshLib/PolygenMesh.h"

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
    GLKLib *pGLK;
    GLKObList polygenMeshList;

private:
    void createActions();
    void createTreeView();
    void createActions_SoftRobotFunction();

    PolygenMesh *getSelectedPolygenMesh();

    QSignalMapper *signalMapper;
    QStandardItemModel *treeModel;

protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dropEvent(QDropEvent *event);

private slots:
    void open();
    void save();
    void signalNavigation(int flag);
    void shiftToOrigin();
    void updateTree();
    void on_pushButton_clearAll_clicked();
    void on_treeView_clicked(const QModelIndex &index);

public slots:
    void soroPreProcessSystem();
    void soroForwardKinmeatics();
    void soroInverseKinmeatics();

private:
    void _inputChamberSelection(QMeshPatch* soroModel);
    void _shirftModeltoXYPlane(QMeshPatch* soroModel);
};

#endif // MAINWINDOW_H
