#ifndef BTVISUALIZER_H
#define BTVISUALIZER_H
#include "IWatcher.h"
#include "../Ros.h"
// #include <QImage>
#include <QLabel>
#include <QPixmap>

class BTVisualizer : public QLabel, public IWatcher{
    Q_OBJECT
public:
    // BTVisualizer(QWidget *parent = nullptr);
    BTVisualizer(QWidget *parent = nullptr);
    ~BTVisualizer() override{};
public slots:
private slots:
protected:
private:
    void Update(const std::string signal) override;
private:
    QPixmap pix;

};

#endif // BTVISUALIZER_H