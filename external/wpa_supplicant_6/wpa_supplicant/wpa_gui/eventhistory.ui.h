

void EventHistory::init()
{
}


void EventHistory::destroy()
{
}


void EventHistory::addEvents(WpaMsgList msgs)
{
    WpaMsgList::iterator it;
    for (it = msgs.begin(); it != msgs.end(); it++) {
	addEvent(*it);
    }
}


void EventHistory::addEvent(WpaMsg msg)
{
    Q3ListViewItem *item;
    item = new Q3ListViewItem(eventListView,
			     msg.getTimestamp().toString("yyyy-MM-dd hh:mm:ss.zzz"),
			     msg.getMsg());
    if (item == NULL)
	return;
    eventListView->setSelected(item, false);
}
