/********************************************************************************
** Form generated from reading UI file 'route_tool.ui'
**
** Created by: Qt User Interface Compiler version 5.15.13
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROUTE_TOOL_H
#define UI_ROUTE_TOOL_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_route_tool
{
public:
    QVBoxLayout *verticalLayout_8;
    QVBoxLayout *verticalLayout;
    QVBoxLayout *verticalLayout_7;
    QTabWidget *tabWidget;
    QWidget *add_tab;
    QGridLayout *gridLayout_2;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QRadioButton *add_node_button;
    QRadioButton *add_edge_button;
    QHBoxLayout *horizontalLayout_4;
    QLabel *add_text;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *add_label_1;
    QTextEdit *add_field_1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *add_label_2;
    QTextEdit *add_field_2;
    QPushButton *create_button;
    QWidget *edit_tab;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_6;
    QRadioButton *edit_node_button;
    QRadioButton *edit_edge_button;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_7;
    QTextEdit *edit_id;
    QHBoxLayout *horizontalLayout_7;
    QLabel *edit_text;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_8;
    QLabel *edit_label_1;
    QTextEdit *edit_field_1;
    QHBoxLayout *horizontalLayout_9;
    QLabel *edit_label_2;
    QTextEdit *edit_field_2;
    QPushButton *confirm_button;
    QHBoxLayout *horizontalLayout_10;
    QWidget *remove_tab;
    QGridLayout *gridLayout_3;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_12;
    QRadioButton *remove_node_button;
    QRadioButton *remove_edge_button;
    QHBoxLayout *horizontalLayout_13;
    QLabel *remove_label;
    QTextEdit *remove_id;
    QPushButton *delete_button;
    QHBoxLayout *horizontalLayout_17;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *load_button;
    QPushButton *save_button;
    QStatusBar *statusbar;

    void setupUi(QWidget *route_tool)
    {
        if (route_tool->objectName().isEmpty())
            route_tool->setObjectName(QString::fromUtf8("route_tool"));
        route_tool->resize(394, 461);
        verticalLayout_8 = new QVBoxLayout(route_tool);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        tabWidget = new QTabWidget(route_tool);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        add_tab = new QWidget();
        add_tab->setObjectName(QString::fromUtf8("add_tab"));
        add_tab->setMinimumSize(QSize(0, 0));
        gridLayout_2 = new QGridLayout(add_tab);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        add_node_button = new QRadioButton(add_tab);
        add_node_button->setObjectName(QString::fromUtf8("add_node_button"));
        add_node_button->setIconSize(QSize(16, 16));

        horizontalLayout->addWidget(add_node_button, 0, Qt::AlignHCenter);

        add_edge_button = new QRadioButton(add_tab);
        add_edge_button->setObjectName(QString::fromUtf8("add_edge_button"));

        horizontalLayout->addWidget(add_edge_button, 0, Qt::AlignHCenter);


        verticalLayout_3->addLayout(horizontalLayout);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        add_text = new QLabel(add_tab);
        add_text->setObjectName(QString::fromUtf8("add_text"));

        horizontalLayout_4->addWidget(add_text, 0, Qt::AlignLeft|Qt::AlignVCenter);


        verticalLayout_3->addLayout(horizontalLayout_4);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        add_label_1 = new QLabel(add_tab);
        add_label_1->setObjectName(QString::fromUtf8("add_label_1"));
        add_label_1->setMaximumSize(QSize(16777215, 50));

        horizontalLayout_3->addWidget(add_label_1);

        add_field_1 = new QTextEdit(add_tab);
        add_field_1->setObjectName(QString::fromUtf8("add_field_1"));
        add_field_1->setMaximumSize(QSize(500, 30));

        horizontalLayout_3->addWidget(add_field_1);


        verticalLayout_2->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        add_label_2 = new QLabel(add_tab);
        add_label_2->setObjectName(QString::fromUtf8("add_label_2"));
        add_label_2->setMaximumSize(QSize(16777215, 50));

        horizontalLayout_2->addWidget(add_label_2);

        add_field_2 = new QTextEdit(add_tab);
        add_field_2->setObjectName(QString::fromUtf8("add_field_2"));
        add_field_2->setMaximumSize(QSize(16777215, 30));

        horizontalLayout_2->addWidget(add_field_2);


        verticalLayout_2->addLayout(horizontalLayout_2);


        verticalLayout_3->addLayout(verticalLayout_2);

        create_button = new QPushButton(add_tab);
        create_button->setObjectName(QString::fromUtf8("create_button"));

        verticalLayout_3->addWidget(create_button, 0, Qt::AlignVCenter);


        gridLayout_2->addLayout(verticalLayout_3, 0, 0, 1, 1);

        tabWidget->addTab(add_tab, QString());
        edit_tab = new QWidget();
        edit_tab->setObjectName(QString::fromUtf8("edit_tab"));
        gridLayout = new QGridLayout(edit_tab);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        edit_node_button = new QRadioButton(edit_tab);
        edit_node_button->setObjectName(QString::fromUtf8("edit_node_button"));
        edit_node_button->setIconSize(QSize(16, 16));

        horizontalLayout_6->addWidget(edit_node_button);

        edit_edge_button = new QRadioButton(edit_tab);
        edit_edge_button->setObjectName(QString::fromUtf8("edit_edge_button"));

        horizontalLayout_6->addWidget(edit_edge_button);


        verticalLayout_4->addLayout(horizontalLayout_6);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QString::fromUtf8("horizontalLayout_11"));
        label_7 = new QLabel(edit_tab);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        horizontalLayout_11->addWidget(label_7);

        edit_id = new QTextEdit(edit_tab);
        edit_id->setObjectName(QString::fromUtf8("edit_id"));
        edit_id->setMaximumSize(QSize(16777215, 50));

        horizontalLayout_11->addWidget(edit_id);


        verticalLayout_4->addLayout(horizontalLayout_11);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        edit_text = new QLabel(edit_tab);
        edit_text->setObjectName(QString::fromUtf8("edit_text"));

        horizontalLayout_7->addWidget(edit_text, 0, Qt::AlignLeft|Qt::AlignVCenter);


        verticalLayout_4->addLayout(horizontalLayout_7);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QString::fromUtf8("horizontalLayout_8"));
        edit_label_1 = new QLabel(edit_tab);
        edit_label_1->setObjectName(QString::fromUtf8("edit_label_1"));

        horizontalLayout_8->addWidget(edit_label_1);

        edit_field_1 = new QTextEdit(edit_tab);
        edit_field_1->setObjectName(QString::fromUtf8("edit_field_1"));
        edit_field_1->setMaximumSize(QSize(16777215, 50));

        horizontalLayout_8->addWidget(edit_field_1);


        verticalLayout_5->addLayout(horizontalLayout_8);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setObjectName(QString::fromUtf8("horizontalLayout_9"));
        edit_label_2 = new QLabel(edit_tab);
        edit_label_2->setObjectName(QString::fromUtf8("edit_label_2"));

        horizontalLayout_9->addWidget(edit_label_2);

        edit_field_2 = new QTextEdit(edit_tab);
        edit_field_2->setObjectName(QString::fromUtf8("edit_field_2"));
        edit_field_2->setMaximumSize(QSize(16777215, 50));

        horizontalLayout_9->addWidget(edit_field_2);


        verticalLayout_5->addLayout(horizontalLayout_9);


        verticalLayout_4->addLayout(verticalLayout_5);

        confirm_button = new QPushButton(edit_tab);
        confirm_button->setObjectName(QString::fromUtf8("confirm_button"));

        verticalLayout_4->addWidget(confirm_button, 0, Qt::AlignVCenter);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QString::fromUtf8("horizontalLayout_10"));

        verticalLayout_4->addLayout(horizontalLayout_10);


        gridLayout->addLayout(verticalLayout_4, 0, 0, 1, 1);

        tabWidget->addTab(edit_tab, QString());
        remove_tab = new QWidget();
        remove_tab->setObjectName(QString::fromUtf8("remove_tab"));
        gridLayout_3 = new QGridLayout(remove_tab);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
        remove_node_button = new QRadioButton(remove_tab);
        remove_node_button->setObjectName(QString::fromUtf8("remove_node_button"));
        remove_node_button->setIconSize(QSize(16, 16));

        horizontalLayout_12->addWidget(remove_node_button);

        remove_edge_button = new QRadioButton(remove_tab);
        remove_edge_button->setObjectName(QString::fromUtf8("remove_edge_button"));

        horizontalLayout_12->addWidget(remove_edge_button);


        verticalLayout_6->addLayout(horizontalLayout_12);

        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QString::fromUtf8("horizontalLayout_13"));
        remove_label = new QLabel(remove_tab);
        remove_label->setObjectName(QString::fromUtf8("remove_label"));

        horizontalLayout_13->addWidget(remove_label);

        remove_id = new QTextEdit(remove_tab);
        remove_id->setObjectName(QString::fromUtf8("remove_id"));
        remove_id->setMaximumSize(QSize(16777215, 50));

        horizontalLayout_13->addWidget(remove_id);


        verticalLayout_6->addLayout(horizontalLayout_13);

        delete_button = new QPushButton(remove_tab);
        delete_button->setObjectName(QString::fromUtf8("delete_button"));

        verticalLayout_6->addWidget(delete_button, 0, Qt::AlignVCenter);

        horizontalLayout_17 = new QHBoxLayout();
        horizontalLayout_17->setObjectName(QString::fromUtf8("horizontalLayout_17"));

        verticalLayout_6->addLayout(horizontalLayout_17);


        gridLayout_3->addLayout(verticalLayout_6, 0, 0, 1, 1);

        tabWidget->addTab(remove_tab, QString());

        verticalLayout_7->addWidget(tabWidget, 0, Qt::AlignHCenter|Qt::AlignTop);


        verticalLayout->addLayout(verticalLayout_7);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        load_button = new QPushButton(route_tool);
        load_button->setObjectName(QString::fromUtf8("load_button"));

        horizontalLayout_5->addWidget(load_button);

        save_button = new QPushButton(route_tool);
        save_button->setObjectName(QString::fromUtf8("save_button"));

        horizontalLayout_5->addWidget(save_button);


        verticalLayout->addLayout(horizontalLayout_5);


        verticalLayout_8->addLayout(verticalLayout);

        statusbar = new QStatusBar(route_tool);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));

        verticalLayout_8->addWidget(statusbar);


        retranslateUi(route_tool);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(route_tool);
    } // setupUi

    void retranslateUi(QWidget *route_tool)
    {
        route_tool->setWindowTitle(QCoreApplication::translate("route_tool", "MainWindow", nullptr));
        add_node_button->setText(QCoreApplication::translate("route_tool", "Node", nullptr));
#if QT_CONFIG(shortcut)
        add_node_button->setShortcut(QCoreApplication::translate("route_tool", "Del", nullptr));
#endif // QT_CONFIG(shortcut)
        add_edge_button->setText(QCoreApplication::translate("route_tool", "Edge", nullptr));
        add_text->setText(QCoreApplication::translate("route_tool", "Text:", nullptr));
        add_label_1->setText(QCoreApplication::translate("route_tool", "Field 1:", nullptr));
        add_label_2->setText(QCoreApplication::translate("route_tool", "Field 2:", nullptr));
        create_button->setText(QCoreApplication::translate("route_tool", "Create", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(add_tab), QCoreApplication::translate("route_tool", "Add", nullptr));
        edit_node_button->setText(QCoreApplication::translate("route_tool", "Node", nullptr));
#if QT_CONFIG(shortcut)
        edit_node_button->setShortcut(QCoreApplication::translate("route_tool", "Del", nullptr));
#endif // QT_CONFIG(shortcut)
        edit_edge_button->setText(QCoreApplication::translate("route_tool", "Edge", nullptr));
        label_7->setText(QCoreApplication::translate("route_tool", "ID:", nullptr));
        edit_text->setText(QCoreApplication::translate("route_tool", "Text:", nullptr));
        edit_label_1->setText(QCoreApplication::translate("route_tool", "Field 1:", nullptr));
        edit_label_2->setText(QCoreApplication::translate("route_tool", "Field 2:", nullptr));
        confirm_button->setText(QCoreApplication::translate("route_tool", "Confirm", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(edit_tab), QCoreApplication::translate("route_tool", "Edit", nullptr));
        remove_node_button->setText(QCoreApplication::translate("route_tool", "Node", nullptr));
#if QT_CONFIG(shortcut)
        remove_node_button->setShortcut(QCoreApplication::translate("route_tool", "Del", nullptr));
#endif // QT_CONFIG(shortcut)
        remove_edge_button->setText(QCoreApplication::translate("route_tool", "Edge", nullptr));
        remove_label->setText(QCoreApplication::translate("route_tool", "ID:", nullptr));
        delete_button->setText(QCoreApplication::translate("route_tool", "Delete", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(remove_tab), QCoreApplication::translate("route_tool", "Remove", nullptr));
        load_button->setText(QCoreApplication::translate("route_tool", "Load", nullptr));
        save_button->setText(QCoreApplication::translate("route_tool", "Save", nullptr));
    } // retranslateUi

};

namespace Ui {
    class route_tool: public Ui_route_tool {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROUTE_TOOL_H
