#ifndef GTKMM_EXAMPLE_MYAREA_H
#define GTKMM_EXAMPLE_MYAREA_H

#include <gtkmm/drawingarea.h>

class MyArea : public Gtk::DrawingArea
{
public:
  MyArea();
  virtual ~MyArea();

protected:
  //Override default signal handler:
  //bool on_draw(int count, const Cairo::RefPtr<Cairo::Context>& cr) override;
  bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
  bool on_timeout();
};

#endif // GTKMM_EXAMPLE_MYAREA_H