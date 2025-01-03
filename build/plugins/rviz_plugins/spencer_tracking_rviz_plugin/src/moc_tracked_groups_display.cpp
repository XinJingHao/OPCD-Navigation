/****************************************************************************
** Meta object code from reading C++ file 'tracked_groups_display.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/plugins/rviz_plugins/spencer_tracking_rviz_plugin/src/tracked_groups_display.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'tracked_groups_display.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_spencer_tracking_rviz_plugin__TrackedGroupsDisplay_t {
    QByteArrayData data[4];
    char stringdata0[90];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_spencer_tracking_rviz_plugin__TrackedGroupsDisplay_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_spencer_tracking_rviz_plugin__TrackedGroupsDisplay_t qt_meta_stringdata_spencer_tracking_rviz_plugin__TrackedGroupsDisplay = {
    {
QT_MOC_LITERAL(0, 0, 50), // "spencer_tracking_rviz_plugin:..."
QT_MOC_LITERAL(1, 51, 23), // "personVisualTypeChanged"
QT_MOC_LITERAL(2, 75, 0), // ""
QT_MOC_LITERAL(3, 76, 13) // "stylesChanged"

    },
    "spencer_tracking_rviz_plugin::TrackedGroupsDisplay\0"
    "personVisualTypeChanged\0\0stylesChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_spencer_tracking_rviz_plugin__TrackedGroupsDisplay[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x08 /* Private */,
       3,    0,   25,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void spencer_tracking_rviz_plugin::TrackedGroupsDisplay::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<TrackedGroupsDisplay *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->personVisualTypeChanged(); break;
        case 1: _t->stylesChanged(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject spencer_tracking_rviz_plugin::TrackedGroupsDisplay::staticMetaObject = { {
    &PersonDisplayCommon<spencer_tracking_msgs::TrackedGroups>::staticMetaObject,
    qt_meta_stringdata_spencer_tracking_rviz_plugin__TrackedGroupsDisplay.data,
    qt_meta_data_spencer_tracking_rviz_plugin__TrackedGroupsDisplay,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *spencer_tracking_rviz_plugin::TrackedGroupsDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *spencer_tracking_rviz_plugin::TrackedGroupsDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_spencer_tracking_rviz_plugin__TrackedGroupsDisplay.stringdata0))
        return static_cast<void*>(this);
    return PersonDisplayCommon<spencer_tracking_msgs::TrackedGroups>::qt_metacast(_clname);
}

int spencer_tracking_rviz_plugin::TrackedGroupsDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = PersonDisplayCommon<spencer_tracking_msgs::TrackedGroups>::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
