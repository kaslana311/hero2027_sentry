#ifndef SET_H
#define SET_H
#include <QDialog>
#include <QMessageBox>

#include "util.hpp"

namespace Ui
{
class set;
}

class set : public QDialog
{
  Q_OBJECT

public:
  explicit set( QWidget * parent = nullptr);
  ~set();
  void UpdateUI(align_res *res);
private slots:
  void on_ok_Button_clicked();
  
  void on_cancel_Button_clicked();

private:
  Ui::set * ui;
  align_res * data;
  
};

#endif  // SET_H
