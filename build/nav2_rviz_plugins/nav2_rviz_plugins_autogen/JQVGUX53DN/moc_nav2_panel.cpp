/****************************************************************************
** Meta object code from reading C++ file 'nav2_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/navigation2/nav2_rviz_plugins/include/nav2_rviz_plugins/nav2_panel.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'nav2_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_nav2_rviz_plugins__Nav2Panel_t {
    QByteArrayData data[21];
    char stringdata0[240];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_nav2_rviz_plugins__Nav2Panel_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_nav2_rviz_plugins__Nav2Panel_t qt_meta_stringdata_nav2_rviz_plugins__Nav2Panel = {
    {
QT_MOC_LITERAL(0, 0, 28), // "nav2_rviz_plugins::Nav2Panel"
QT_MOC_LITERAL(1, 29, 11), // "startThread"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 9), // "onStartup"
QT_MOC_LITERAL(4, 52, 10), // "onShutdown"
QT_MOC_LITERAL(5, 63, 8), // "onCancel"
QT_MOC_LITERAL(6, 72, 7), // "onPause"
QT_MOC_LITERAL(7, 80, 8), // "onResume"
QT_MOC_LITERAL(8, 89, 11), // "onResumedWp"
QT_MOC_LITERAL(9, 101, 15), // "onAccumulatedWp"
QT_MOC_LITERAL(10, 117, 16), // "onAccumulatedNTP"
QT_MOC_LITERAL(11, 134, 14), // "onAccumulating"
QT_MOC_LITERAL(12, 149, 9), // "onNewGoal"
QT_MOC_LITERAL(13, 159, 1), // "x"
QT_MOC_LITERAL(14, 161, 1), // "y"
QT_MOC_LITERAL(15, 163, 5), // "theta"
QT_MOC_LITERAL(16, 169, 5), // "frame"
QT_MOC_LITERAL(17, 175, 15), // "handleGoalSaver"
QT_MOC_LITERAL(18, 191, 16), // "handleGoalLoader"
QT_MOC_LITERAL(19, 208, 11), // "loophandler"
QT_MOC_LITERAL(20, 220, 19) // "initialStateHandler"

    },
    "nav2_rviz_plugins::Nav2Panel\0startThread\0"
    "\0onStartup\0onShutdown\0onCancel\0onPause\0"
    "onResume\0onResumedWp\0onAccumulatedWp\0"
    "onAccumulatedNTP\0onAccumulating\0"
    "onNewGoal\0x\0y\0theta\0frame\0handleGoalSaver\0"
    "handleGoalLoader\0loophandler\0"
    "initialStateHandler"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_nav2_rviz_plugins__Nav2Panel[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x08 /* Private */,
       3,    0,   90,    2, 0x08 /* Private */,
       4,    0,   91,    2, 0x08 /* Private */,
       5,    0,   92,    2, 0x08 /* Private */,
       6,    0,   93,    2, 0x08 /* Private */,
       7,    0,   94,    2, 0x08 /* Private */,
       8,    0,   95,    2, 0x08 /* Private */,
       9,    0,   96,    2, 0x08 /* Private */,
      10,    0,   97,    2, 0x08 /* Private */,
      11,    0,   98,    2, 0x08 /* Private */,
      12,    4,   99,    2, 0x08 /* Private */,
      17,    0,  108,    2, 0x08 /* Private */,
      18,    0,  109,    2, 0x08 /* Private */,
      19,    0,  110,    2, 0x08 /* Private */,
      20,    0,  111,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::QString,   13,   14,   15,   16,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void nav2_rviz_plugins::Nav2Panel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<Nav2Panel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->startThread(); break;
        case 1: _t->onStartup(); break;
        case 2: _t->onShutdown(); break;
        case 3: _t->onCancel(); break;
        case 4: _t->onPause(); break;
        case 5: _t->onResume(); break;
        case 6: _t->onResumedWp(); break;
        case 7: _t->onAccumulatedWp(); break;
        case 8: _t->onAccumulatedNTP(); break;
        case 9: _t->onAccumulating(); break;
        case 10: _t->onNewGoal((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 11: _t->handleGoalSaver(); break;
        case 12: _t->handleGoalLoader(); break;
        case 13: _t->loophandler(); break;
        case 14: _t->initialStateHandler(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject nav2_rviz_plugins::Nav2Panel::staticMetaObject = { {
    QMetaObject::SuperData::link<rviz_common::Panel::staticMetaObject>(),
    qt_meta_stringdata_nav2_rviz_plugins__Nav2Panel.data,
    qt_meta_data_nav2_rviz_plugins__Nav2Panel,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *nav2_rviz_plugins::Nav2Panel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *nav2_rviz_plugins::Nav2Panel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_nav2_rviz_plugins__Nav2Panel.stringdata0))
        return static_cast<void*>(this);
    return rviz_common::Panel::qt_metacast(_clname);
}

int nav2_rviz_plugins::Nav2Panel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rviz_common::Panel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}
struct qt_meta_stringdata_nav2_rviz_plugins__InitialThread_t {
    QByteArrayData data[6];
    char stringdata0[110];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_nav2_rviz_plugins__InitialThread_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_nav2_rviz_plugins__InitialThread_t qt_meta_stringdata_nav2_rviz_plugins__InitialThread = {
    {
QT_MOC_LITERAL(0, 0, 32), // "nav2_rviz_plugins::InitialThread"
QT_MOC_LITERAL(1, 33, 16), // "navigationActive"
QT_MOC_LITERAL(2, 50, 0), // ""
QT_MOC_LITERAL(3, 51, 18), // "navigationInactive"
QT_MOC_LITERAL(4, 70, 18), // "localizationActive"
QT_MOC_LITERAL(5, 89, 20) // "localizationInactive"

    },
    "nav2_rviz_plugins::InitialThread\0"
    "navigationActive\0\0navigationInactive\0"
    "localizationActive\0localizationInactive"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_nav2_rviz_plugins__InitialThread[] = {

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

void nav2_rviz_plugins::InitialThread::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<InitialThread *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->navigationActive(); break;
        case 1: _t->navigationInactive(); break;
        case 2: _t->localizationActive(); break;
        case 3: _t->localizationInactive(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (InitialThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialThread::navigationActive)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (InitialThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialThread::navigationInactive)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (InitialThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialThread::localizationActive)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (InitialThread::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&InitialThread::localizationInactive)) {
                *result = 3;
                return;
            }
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject nav2_rviz_plugins::InitialThread::staticMetaObject = { {
    QMetaObject::SuperData::link<QThread::staticMetaObject>(),
    qt_meta_stringdata_nav2_rviz_plugins__InitialThread.data,
    qt_meta_data_nav2_rviz_plugins__InitialThread,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *nav2_rviz_plugins::InitialThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *nav2_rviz_plugins::InitialThread::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_nav2_rviz_plugins__InitialThread.stringdata0))
        return static_cast<void*>(this);
    return QThread::qt_metacast(_clname);
}

int nav2_rviz_plugins::InitialThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
void nav2_rviz_plugins::InitialThread::navigationActive()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void nav2_rviz_plugins::InitialThread::navigationInactive()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void nav2_rviz_plugins::InitialThread::localizationActive()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void nav2_rviz_plugins::InitialThread::localizationInactive()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
