#include <gtkmm.h>
#include <cmath>

//int main(int argc, char* argv[]){
 // Gtk::Main kitt(argc, argv);
  
  //Gtk::Window window;
  //Gtk::DrawingArea circle;
  
  //Gdk::Window::create_cairo_context()
 // circle.arc(1, 1, 1, 1, 1, 2.0*M_PI)
  //window.add(circle);
  

  
  
    // now draw a circle
  //cr->save();
  //arc(1, 1, 1 / 4.0, 0.0, 2.0 * M_PI); // full circle
  //cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
  //cr->fill_preserve();
  //cr->restore();  // back to opaque black
  //cr->stroke();
  

  
  //  Gtk::Main::run(window);
  

bool MyArea::on_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
  // This is where we draw on the window
  Gtk::Allocation allocation = get_allocation();
  const int width = allocation.get_width();
  const int height = allocation.get_height();
  const int lesser = MIN(width, height);

  // coordinates for the center of the window
  int xc, yc;
  xc = width / 2;
  yc = height / 2;

  cr->set_line_width(lesser * 0.02);  // outline thickness changes
                                      // with window size

  // first draw a simple unclosed arc
  cr->save();
  cr->arc(width / 3.0, height / 4.0, lesser / 4.0, -(M_PI / 5.0), M_PI);
  cr->close_path();   // line back to start point
  cr->set_source_rgb(0.0, 0.8, 0.0);
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();   // outline it

  // now draw a circle
  cr->save();
  cr->arc(xc, yc, lesser / 4.0, 0.0, 2.0 * M_PI); // full circle
  cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();

  // and finally an ellipse
  double ex, ey, ew, eh;
  // center of ellipse
  ex = xc;
  ey = 3.0 * height / 4.0;
  // ellipse dimensions
  ew = 3.0 * width / 4.0;
  eh = height / 3.0;

  cr->save();

  cr->translate(ex, ey);  // make (ex, ey) == (0, 0)
  cr->scale(ew / 2.0, eh / 2.0);  // for width: ew / 2.0 == 1.0
                                  // for height: eh / 2.0 == 1.0

  cr->arc(0.0, 0.0, 1.0, 0.0, 2 * M_PI);  // 'circle' centered at (0, 0)
                                          // with 'radius' of 1.0

  cr->set_source_rgba(0.8, 0.0, 0.0, 0.7);
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();

  return true;
}

  
//return 0;
//}