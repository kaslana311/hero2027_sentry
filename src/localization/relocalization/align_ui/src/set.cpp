#include "set.h"

#include "ui_set.h"
#define M_PI 3.14159265358979323846
set::set( QWidget* parent)
    : QDialog(parent), ui(new Ui::set) {
    ui->setupUi(this);
    ui->x_lineEdit->setText(QString::number(-4.6));
    ui->y_lineEdit->setText(QString::number(3.7));
    ui->z_lineEdit->setText(QString::number(0));
    ui->roll_lineEdit->setText(QString::number(0));
    ui->pitch_lineEdit->setText(QString::number(0));
    ui->yaw_lineEdit->setText(QString::number(0));
}

set::~set() { delete ui; }
void set::UpdateUI(align_res *res) { 
    data=res;
}
void set::on_ok_Button_clicked() {
    data->flag = 0;
    QString x_str = ui->x_lineEdit->text();
    QString y_str = ui->y_lineEdit->text();
    QString z_str = ui->z_lineEdit->text();
    QString roll_str = ui->roll_lineEdit->text();
    QString pitch_str = ui->pitch_lineEdit->text();
    QString yaw_str = ui->yaw_lineEdit->text();
    if (x_str.isEmpty() || y_str.isEmpty() || z_str.isEmpty() ||
        roll_str.isEmpty() || pitch_str.isEmpty() || yaw_str.isEmpty()) {
        QMessageBox::warning(this, "Warning", "小子,别漏掉参数!!!.");
        return;
    }
    data->x = x_str.toDouble();
    data->y = y_str.toDouble();
    data->z = z_str.toDouble();
    data->roll = roll_str.toDouble() / 180.0 * M_PI;
    data->pitch = pitch_str.toDouble() / 180.0 * M_PI;
    data->yaw = yaw_str.toDouble() / 180.0 * M_PI;
    data->flag = 1;
    QMessageBox::information(this, "Info", "我会出手!");
    this->close();
}

void set::on_cancel_Button_clicked() {
    data->flag = 0;
    this->close();
}
