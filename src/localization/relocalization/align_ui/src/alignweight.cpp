#include "alignweight.h"

#include "ui_alignweight.h"
Q_DECLARE_METATYPE(align_res)
Q_DECLARE_METATYPE(reloc_res)
Alignweight::Alignweight(int argc, char** argv, QWidget* parent)
    : QWidget(parent), ui(new Ui::Alignweight), argc_(argc), argv_(argv) {
    ui->setupUi(this);
    set_ = new set( this);
    rclcpp::init(argc_, argv_);
    qRegisterMetaType<align_res>("align_res");
    qRegisterMetaType<reloc_res>("reloc_res");
    this->relocalization_node_ = std::make_shared<RelocalizationNode>();
    connect(relocalization_node_.get(), &RelocalizationNode::alignResultSignal,
            this,
            &Alignweight::AlignResCBK);  //匹配结果显示
    connect(relocalization_node_.get(), &RelocalizationNode::setResultSignal,
            this,
            &Alignweight::SetResCBK);  //设置结果反馈
    connect(this, &Alignweight::AlignWeightRetry, relocalization_node_.get(),
            &RelocalizationNode::OnRetrySet);  //重试信息发布
    connect(this, &Alignweight::AlignWeightSet, relocalization_node_.get(),
            &RelocalizationNode::onSetAlignSignal);  //设置信息发布
    align_thread_ = new QThread(this);
    relocalization_node_->moveToThread(align_thread_);
    connect(align_thread_, &QThread::started, relocalization_node_.get(),
            [this]() {
                std::thread([this]() {
                    rclcpp::spin(this->relocalization_node_);
                }).detach();
            });
    align_thread_->start();
}

Alignweight::~Alignweight() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();  // 停止所有 spin
    }
    if (align_thread_ && align_thread_->isRunning()) {
        align_thread_->quit();
        align_thread_->wait();
    }

    delete align_thread_;
    delete ui;
}

void Alignweight::on_retry_Button_clicked() {
    this->ui->label_2->clear();
    QMessageBox::information(this, "Info", "把嘎,这结果大大滴不好,再来一次");
    emit AlignWeightRetry(true);
}

void Alignweight::on_set_Button_clicked() {
    this->ui->label_2->clear();
    align_res res;
    
    set_->UpdateUI(&res);
    set_->setModal(true);
    set_->exec();
    if (res.flag == 0) return;
    emit AlignWeightSet(res);
    this->ui->label_2->setText("等待参数设置反馈,5s未受到请重新发布!");
}

void Alignweight::on_ok_Button_clicked() {
    this->ui->label_2->clear();
    QMessageBox::information(this, "Info", "结果不错,我就要这个,程序即将关闭!");
    this->close();
}
void Alignweight::AlignResCBK(reloc_res res) {
    if (res.isOk) {
        QString info = QString(
                           "配准成功!\n匹配得分: %1\n平移变化: %2 m\n滚转变化: "
                           "%3 deg\n俯仰变化: %4 "
                           "deg\n偏航变化: %5 deg")
                           .arg(res.fitness)
                           .arg(res.dtrans)
                           .arg(res.droll * 180.0 / M_PI)
                           .arg(res.dpitch * 180.0 / M_PI)
                           .arg(res.dyaw * 180.0 / M_PI);
        this->ui->label_2->setText(info);
    } else {
        QString info = QString(
                           "配准失败!\n匹配得分: %1\n平移变化: %2 m\n滚转变化: "
                           "%3 deg\n俯仰变化: %4 "
                           "deg\n偏航变化: %5 deg")
                           .arg(res.fitness)
                           .arg(res.dtrans)
                           .arg(res.droll * 180.0 / M_PI)
                           .arg(res.dpitch * 180.0 / M_PI)
                           .arg(res.dyaw * 180.0 / M_PI);
        this->ui->label_2->setText(info);
    }
}
void Alignweight::SetResCBK(bool isOk) {
    if (isOk) {
        QMessageBox::information(this, "Info", "参数设置成功!");
        this->ui->label_2->setText("参数设置成功!,请查看rviz2效果!");
        // this->close();
    } else {
        QMessageBox::warning(this, "Warning", "参数设置失败!,再试试");
    }
}