#if (defined(__VXWORKS__) && defined(__VXWORKS7__)) || !defined(__VXWORKS__)
  #pragma pack(push,4)
#endif

#if defined _MSC_VER
  //Disable warning C4103: alignment changed after including header, may be due to missing #pragma pack(pop)
  #pragma warning(disable:4103)

  //Disable warning C4201: nonstandard extension used : nameless struct/union
  #pragma warning(disable:4201)
#endif
