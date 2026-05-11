#ifndef ALIGNWEIGHT_H
#define ALIGNWEIGHT_H

#include <QMessageBox>
#include <QThread>
#include <QWidget>
// #include <thread>
#include <QTimer>

#include "relocalization_node.hpp"
#include "set.h"
#include "util.hpp"
QT_BEGIN_NAMESPACE
namespace Ui
{
class Alignweight;
}
QT_END_NAMESPACE

class Alignweight : public QWidget
{
  Q_OBJECT

public:
  Alignweight(int argc, char ** argv, QWidget * parent = nullptr);
  ~Alignweight();

private slots:
  void on_retry_Button_clicked();

  void on_set_Button_clicked();

  void on_ok_Button_clicked();
  void AlignResCBK(reloc_res res);
  void SetResCBK(bool isOk);
signals:
  void AlignWeightSet(align_res res);
  void AlignWeightRetry(bool isOk);

private:
  Ui::Alignweight * ui;
  std::shared_ptr<RelocalizationNode> relocalization_node_;
  QThread * align_thread_;
  int argc_;
  char ** argv_;
  set * set_;
};
#endif  // ALIGNWEIGHT_H
