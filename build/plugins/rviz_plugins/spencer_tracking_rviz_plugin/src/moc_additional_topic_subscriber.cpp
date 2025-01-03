/****************************************************************************
** Meta object code from reading C++ file 'additional_topic_subscriber.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/plugins/rviz_plugins/spencer_tracking_rviz_plugin/src/additional_topic_subscriber.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'additional_topic_subscriber.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rviz___AdditionalTopicSubscriber_t {
    QByteArrayData data[5];
    char stringdata0[85];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rviz___AdditionalTopicSubscriber_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rviz___AdditionalTopicSubscriber_t qt_meta_stringdata_rviz___AdditionalTopicSubscriber = {
    {
QT_MOC_LITERAL(0, 0, 32), // "rviz::_AdditionalTopicSubscriber"
QT_MOC_LITERAL(1, 33, 11), // "updateTopic"
QT_MOC_LITERAL(2, 45, 0), // ""
QT_MOC_LITERAL(3, 46, 20), // "displayEnableChanged"
QT_MOC_LITERAL(4, 67, 17) // "fixedFrameChanged"

    },
    "rviz::_AdditionalTopicSubscriber\0"
    "updateTopic\0\0displayEnableChanged\0"
    "fixedFrameChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rviz___AdditionalTopicSubscriber[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x09 /* Protected */,
       3,    0,   30,    2, 0x09 /* Protected */,
       4,    0,   31,    2, 0x09 /* Protected */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void rviz::_AdditionalTopicSubscriber::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<_AdditionalTopicSubscriber *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->updateTopic(); break;
        case 1: _t->displayEnableChanged(); break;
        case 2: _t->fixedFrameChanged(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject rviz::_AdditionalTopicSubscriber::staticMetaObject = { {
    &QObject::staticMetaObject,
    qt_meta_stringdata_rviz___AdditionalTopicSubscriber.data,
    qt_meta_data_rviz___AdditionalTopicSubscriber,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *rviz::_AdditionalTopicSubscriber::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rviz::_AdditionalTopicSubscriber::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rviz___AdditionalTopicSubscriber.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int rviz::_AdditionalTopicSubscriber::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
