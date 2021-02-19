#ifndef CONTACT_JOINT_TRAJECTORY_SEGMENT_H
#define CONTACT_JOINT_TRAJECTORY_SEGMENT_H

class ContactSegment {
public:
  typedef bool Contact;
  typedef double Time;

  ContactSegment(void) : contacts(false), time_from_start(0.0){};

  ContactSegment(Contact &_contacts, Time time)
      : contacts(_contacts), time_from_start(time) {}

  ~ContactSegment() {}

  Time getTime(void) const { return time_from_start; }

  const Contact &getContact(void) const { return contacts; }

private:
  Contact contacts;
  Time time_from_start;
};

#endif /* CONTACT_JOINT_TRAJECTORY_SEGMENT_H */
