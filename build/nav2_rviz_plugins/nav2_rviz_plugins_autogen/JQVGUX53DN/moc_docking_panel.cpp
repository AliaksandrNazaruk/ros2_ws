/****************************************************************************
** Meta object code from reading C++ file 'docking_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/navigation2/nav2_rviz_plugins/include/nav2_rviz_plugins/docking_panel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'docking_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_nav2_rviz_plugins__DockingPanel_t {
    QByteArrayData data[9];
    char stringdata0[152];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_nav2_rviz_plugins__DockingPanel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_nav2_rviz_plugins__DockingPanel_t qt_meta_stringdata_nav2_rviz_plugins__DockingPanel = {
    {
QT_MOC_LITERAL(0, 0, 31), // "nav2_rviz_plugins::DockingPanel"
QT_MOC_LITERAL(1, 32, 11), // "startThread"
QT_MOC_LITERAL(2, 44, 0), // ""
QT_MOC_LITERAL(3, 45, 9), // "onStartup"
QT_MOC_LITERAL(4, 55, 22), // "onDockingButtonPressed"
QT_MOC_LITERAL(5, 78, 24), // "onUndockingButtonPressed"
QT_MOC_LITERAL(6, 103, 15), // "onCancelDocking"
QT_MOC_LITERAL(7, 119, 17), // "onCancelUndocking"
QT_MOC_LITERAL(8, 137, 14) // "dockIdCheckbox"

    },
    "nav2_rviz_plugins::DockingPanel\0"
    "startThread\0\0onStartup\0onDockingButtonPressed\0"
    "onUndockingButtonPressed\0onCancelDocking\0"
    "onCancelUndocking\0dockIdCheckbox"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_nav2_rviz_plugins__DockingPanel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x08 /* Private */,
       3,    0,   50,    2, 0x08 /* Private */,
       4,    0,   51,    2, 0x08 /* Private */,
       5,    0,   52,    2, 0x08 /* Private */,
       6,    0,   53,    2, 0x08 /* Private */,
       7,    0,   54,    2, 0x08 /* Private */,
       8,    0,   55,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void nav2_rviz_plugins::DockingPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DockingPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->startThread(); break;
        case 1: _t->onStartup(); break;
        case 2: _t->onDockingButtonPressed(); break;
        case 3: _t->onUndockingButtonPressed(); break;
        case 4: _t->onCancelDocking(); break;
        case 5: _t->onCancelUndocking(); break;
        case 6: _t->dockIdCheckbox(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject nav2_rviz_plugins::DockingPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_nav2_rviz_plugins__DockingPanel.data,
    qt_meta_data_nav2_rviz_plugins__DockingPanel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *nav2_rviz_plugins::DockingPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *nav2_rviz_plugins::DockingPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_nav2_rviz_plugins__DockingPanel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int nav2_rviz_plugins::DockingPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
struct qt_meta_stringdata_nav2_rviz_plugins__InitialDockThread_t {
    QByteArrayData data[6];
    char stringdata0[102];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_nav2_rviz_plugins__InitialDockThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_nav2_rviz_plugins__InitialDockThread_t qt_meta_stringdata_nav2_rviz_plugins__InitialDockThread = {
    {
QT_MOC_LITERAL(0, 0, 36), // "nav2_rviz_plugins::InitialDoc..."
QT_MOC_LITERAL(1, 37, 13), // "dockingActive"
QT_MOC_LITERAL(2, 51, 0), // ""
QT_MOC_LITERAL(3, 52, 15), // "dockingInactive"
QT_MOC_LITERAL(4, 68, 15), // "undockingActive"
QT_MOC_LITERAL(5, 84, 17) // "undockingInactive"

    },
    "nav2_rviz_plugins::InitialDockThread\0"
    "dockingActive\0\0dockingInactive\0"
    "undockingActive\0undockingInactive"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_nav2_rviz_plugins__InitialDockThread[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x06 /* Public */,
       3,    0,   35,    2, 0x06 /* Public */,
       4,    0,   36,    2, 0x06 /* Public */,
       5,    0,   37,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void nav2_rviz_plugins::InitialDockThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<InitialDockThread *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->dockingActive(); break;
        case 1: _t->dockingInactive(); break;
        case 2: _t->undockingActive(); break;
        case 3: _t->undockingInactive(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (InitialDockThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialDockThread::dockingActive)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (InitialDockThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialDockThread::dockingInactive)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (InitialDockThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialDockThread::undockingActive)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (InitialDockThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialDockThread::undockingInactive)) {
                *result = 3;
                return;
            }
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject nav2_rviz_plugins::InitialDockThread::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_nav2_rviz_plugins__InitialDockThread.data,
    qt_meta_data_nav2_rviz_plugins__InitialDockThread,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *nav2_rviz_plugins::InitialDockThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *nav2_rviz_plugins::InitialDockThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_nav2_rviz_plugins__InitialDockThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int nav2_rviz_plugins::InitialDockThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void nav2_rviz_plugins::InitialDockThread::dockingActive()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void nav2_rviz_plugins::InitialDockThread::dockingInactive()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void nav2_rviz_plugins::InitialDockThread::undockingActive()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void nav2_rviz_plugins::InitialDockThread::undockingInactive()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
