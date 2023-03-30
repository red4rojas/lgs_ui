#include "crawler_panel.h"
#include "module_button.h"
#include <QHBoxLayout>
#include <QWidget>

CrawlerPanel::CrawlerPanel(QWidget *parent): QWidget(parent){
    auto front_grip = new ModuleButton(std::string("front_grip_on"), std::string("front_grip_off"));
    auto back_grip = new ModuleButton(std::string("back_grip_on"), std::string("back_grip_off"));
    auto extenders = new ModuleButton(std::string("extender_on"), std::string("extender_off"));
    auto forward = new ModuleButton(std::string("forward"), std::string("stop"));
    auto backward = new ModuleButton(std::string("backward"), std::string("stop"));
    auto layout = new QHBoxLayout();
    layout->addWidget(forward);
    layout->addWidget(front_grip);
    layout->addWidget(extenders);
    layout->addWidget(back_grip);
    layout->addWidget(backward);
    setLayout(layout);
}

CrawlerPanel::~CrawlerPanel(){
}
 