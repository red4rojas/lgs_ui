#ifndef STATUSBAR_H
#define STATUSBAR_H
#include "IWatcher.h"
#include "../Ros.h"
#include <QStatusBar>


class StatusBar : public QStatusBar, public IWatcher{
    Q_OBJECT
public:
    StatusBar(QWidget *parent = nullptr);
    ~StatusBar() override{};
public slots:
private slots:
protected:
private:
    void Update(const std::string signal) override;
private:

};

#endif // STATUSBAR_H