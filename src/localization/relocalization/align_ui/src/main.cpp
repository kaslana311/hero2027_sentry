#include <QApplication>

#include "alignweight.h"

int main(int argc, char * argv[])
{
  QApplication a(argc, argv);
  Alignweight w(argc, argv);
  w.show();
  return a.exec();
}
