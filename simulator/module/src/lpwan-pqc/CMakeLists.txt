build_lib(
  LIBNAME lpwan-pqc

  SOURCE_FILES

    # Application
    model/pqc-exchange-application.cc

    # Models
    model/lpwan-packet-error-model.cc
    model/lpwan-aloha-collision-model.cc
    model/lpwan-lora-airtime.cc

    # Helper
    helper/lpwan-pqc-helper.cc


  HEADER_FILES

    # Application
    model/pqc-exchange-application.h

    # Models
    model/lpwan-packet-error-model.h
    model/lpwan-aloha-collision-model.h
    model/lpwan-lora-airtime.h
    # Helper
    helper/lpwan-pqc-helper.h


  LIBRARIES_TO_LINK

    ${libcore}
    ${libnetwork}
    ${libapplications}
)
