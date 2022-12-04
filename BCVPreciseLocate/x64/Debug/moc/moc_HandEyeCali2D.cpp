/****************************************************************************
** Meta object code from reading C++ file 'HandEyeCali2D.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../HandEyeCali2D.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'HandEyeCali2D.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_THandEyeCali2D_t {
    QByteArrayData data[9];
    char stringdata0[155];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_THandEyeCali2D_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_THandEyeCali2D_t qt_meta_stringdata_THandEyeCali2D = {
    {
QT_MOC_LITERAL(0, 0, 14), // "THandEyeCali2D"
QT_MOC_LITERAL(1, 15, 22), // "restart_frmmain_stream"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 12), // "set_exposure"
QT_MOC_LITERAL(4, 52, 8), // "set_gain"
QT_MOC_LITERAL(5, 61, 21), // "on_btn_get_IR_clicked"
QT_MOC_LITERAL(6, 83, 22), // "on_btn_capture_clicked"
QT_MOC_LITERAL(7, 106, 24), // "on_btn_calculate_clicked"
QT_MOC_LITERAL(8, 131, 23) // "on_btn_show_TCP_clicked"

    },
    "THandEyeCali2D\0restart_frmmain_stream\0"
    "\0set_exposure\0set_gain\0on_btn_get_IR_clicked\0"
    "on_btn_capture_clicked\0on_btn_calculate_clicked\0"
    "on_btn_show_TCP_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_THandEyeCali2D[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   49,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   50,    2, 0x08 /* Private */,
       4,    0,   51,    2, 0x08 /* Private */,
       5,    0,   52,    2, 0x08 /* Private */,
       6,    0,   53,    2, 0x08 /* Private */,
       7,    0,   54,    2, 0x08 /* Private */,
       8,    0,   55,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void THandEyeCali2D::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<THandEyeCali2D *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->restart_frmmain_stream(); break;
        case 1: _t->set_exposure(); break;
        case 2: _t->set_gain(); break;
        case 3: _t->on_btn_get_IR_clicked(); break;
        case 4: _t->on_btn_capture_clicked(); break;
        case 5: _t->on_btn_calculate_clicked(); break;
        case 6: _t->on_btn_show_TCP_clicked(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (THandEyeCali2D::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&THandEyeCali2D::restart_frmmain_stream)) {
                *result = 0;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject THandEyeCali2D::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_THandEyeCali2D.data,
    qt_meta_data_THandEyeCali2D,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *THandEyeCali2D::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *THandEyeCali2D::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_THandEyeCali2D.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int THandEyeCali2D::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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

// SIGNAL 0
void THandEyeCali2D::restart_frmmain_stream()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
