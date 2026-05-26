#include "set.h"

#include <QHBoxLayout>
#include <QPushButton>
#include "ui_set.h"
#define M_PI 3.14159265358979323846
set::set( QWidget* parent)
    : QDialog(parent), ui(new Ui::set) {
    ui->setupUi(this);

    ui->x_lineEdit->setText(QString::number(4.065, 'f', 3));
    ui->y_lineEdit->setText(QString::number(7.5, 'f', 3));
    ui->z_lineEdit->setText(QString::number(0.000, 'f', 3));
    ui->roll_lineEdit->setText(QString::number(0.000, 'f', 3));
    ui->pitch_lineEdit->setText(QString::number(0.000, 'f', 3));
    ui->yaw_lineEdit->setText(QString::number(0.000, 'f', 3));

    // Increase the width of the layout container so buttons don't get squished
    if (QWidget* container = ui->x_lineEdit->parentWidget()) {
        container->setMinimumWidth(550);
        this->setMinimumWidth(800);
    }

    auto setup_buttons = [this](QHBoxLayout* layout, QLineEdit* lineEdit) {
         lineEdit->setAlignment(Qt::AlignLeft);
        QPushButton* m_01 = new QPushButton("-0.1", this);
        QPushButton* p_01 = new QPushButton("+0.1", this);
        QPushButton* m_001 = new QPushButton("-0.01", this);
        QPushButton* p_001 = new QPushButton("+0.01", this);
        
        m_01->setMaximumWidth(45);
        p_01->setMaximumWidth(45);
        m_001->setMaximumWidth(55);
        p_001->setMaximumWidth(55);

        layout->addWidget(m_01);
        layout->addWidget(p_01);
        layout->addWidget(m_001);
        layout->addWidget(p_001);
        connect(m_01, &QPushButton::clicked, [lineEdit]() { lineEdit->setText(QString::number(lineEdit->text().toDouble() - 0.1, 'f', 3)); });
        connect(p_01, &QPushButton::clicked, [lineEdit]() { lineEdit->setText(QString::number(lineEdit->text().toDouble() + 0.1, 'f', 3)); });
        connect(m_001, &QPushButton::clicked, [lineEdit]() { lineEdit->setText(QString::number(lineEdit->text().toDouble() - 0.01, 'f', 3)); });
        connect(p_001, &QPushButton::clicked, [lineEdit]() { lineEdit->setText(QString::number(lineEdit->text().toDouble() + 0.01, 'f', 3)); });
    };

    setup_buttons(ui->horizontalLayout, ui->x_lineEdit);
    setup_buttons(ui->horizontalLayout_2, ui->y_lineEdit);
    setup_buttons(ui->horizontalLayout_3, ui->z_lineEdit);
    setup_buttons(ui->horizontalLayout_4, ui->roll_lineEdit);
    setup_buttons(ui->horizontalLayout_5, ui->pitch_lineEdit);
    setup_buttons(ui->horizontalLayout_6, ui->yaw_lineEdit);
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
