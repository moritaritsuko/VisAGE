/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../qt/mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      34,   11,   11,   11, 0x08,
      57,   11,   11,   11, 0x08,
      79,   11,   11,   11, 0x08,
     102,   11,   11,   11, 0x08,
     129,   11,   11,   11, 0x08,
     153,   11,   11,   11, 0x08,
     178,   11,   11,   11, 0x08,
     202,   11,   11,   11, 0x08,
     227,   11,   11,   11, 0x08,
     252,   11,   11,   11, 0x08,
     273,   11,   11,   11, 0x08,
     294,   11,   11,   11, 0x08,
     317,   11,   11,   11, 0x08,
     342,   11,   11,   11, 0x08,
     363,   11,   11,   11, 0x08,
     391,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0on_btnRodar_clicked()\0"
    "on_btnManual_clicked()\0on_btnMover_clicked()\0"
    "on_btnStereo_clicked()\0"
    "on_btnRotacionar_clicked()\0"
    "on_btnGAMAGON_clicked()\0"
    "on_btnGAMAGOFF_clicked()\0"
    "on_btnCaptura_clicked()\0"
    "on_btnPararCap_clicked()\0"
    "on_btnCalibStr_clicked()\0on_btnDsip_clicked()\0"
    "on_btnFoco_clicked()\0on_btnSalvar_clicked()\0"
    "on_btninVision_clicked()\0on_btnIV_2_clicked()\0"
    "on_btnCapturaMono_clicked()\0"
    "on_MainWindow_destroyed()\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_btnRodar_clicked(); break;
        case 1: _t->on_btnManual_clicked(); break;
        case 2: _t->on_btnMover_clicked(); break;
        case 3: _t->on_btnStereo_clicked(); break;
        case 4: _t->on_btnRotacionar_clicked(); break;
        case 5: _t->on_btnGAMAGON_clicked(); break;
        case 6: _t->on_btnGAMAGOFF_clicked(); break;
        case 7: _t->on_btnCaptura_clicked(); break;
        case 8: _t->on_btnPararCap_clicked(); break;
        case 9: _t->on_btnCalibStr_clicked(); break;
        case 10: _t->on_btnDsip_clicked(); break;
        case 11: _t->on_btnFoco_clicked(); break;
        case 12: _t->on_btnSalvar_clicked(); break;
        case 13: _t->on_btninVision_clicked(); break;
        case 14: _t->on_btnIV_2_clicked(); break;
        case 15: _t->on_btnCapturaMono_clicked(); break;
        case 16: _t->on_MainWindow_destroyed(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
