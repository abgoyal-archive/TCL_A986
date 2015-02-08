

#ifndef ADDINTERFACE_H
#define ADDINTERFACE_H

#include <QObject>

#include <QtGui/QDialog>
#include <QtGui/QTreeWidget>
#include <QtGui/QVBoxLayout>

class WpaGui;

class AddInterface : public QDialog
{
	Q_OBJECT

public:
	AddInterface(WpaGui *_wpagui, QWidget *parent = 0);

public slots:
	virtual void interfaceSelected(QTreeWidgetItem *sel);

private:
	void addInterfaces();
	bool addRegistryInterface(const QString &ifname);

	QVBoxLayout *vboxLayout;
	QTreeWidget *interfaceWidget;
	WpaGui *wpagui;
};

#endif /* ADDINTERFACE_H */
