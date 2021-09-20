#ifndef LXS_LIB_SERIALIZATION_H
#define LXS_LIB_SERIALIZATION_H

#include <stdint.h>
#include <exception>
#include <stdexcept>

namespace LxS {

    class SerializationOverflowError : public std::overflow_error {
        public:
            SerializationOverflowError() : std::overflow_error("serialization overflow") {}
    };

    class Serializable {
        protected:
            template <class iterator> 
                void pushb(iterator & it, const iterator & end, uint8_t val) const {
                    if (it == end) {
                        throw SerializationOverflowError();
                    }
                    *it = val;
                    it ++;
                }

            template <class iterator> 
                uint8_t readb(iterator & it, const iterator & end) const {
                    if (it == end) {
                        throw SerializationOverflowError();
                    }
                    return *it++;
                }

            template <class iterator> 
                void pushw(iterator & it, const iterator & end, uint16_t val) const {
                    pushb(it,end,val & 0xFF);
                    pushb(it,end,(val>>8) & 0xFF);
                }

            template <class iterator> 
                uint16_t readw(iterator & it, const iterator & end) const {
                    uint16_t ret = readb(it,end);
                    ret |= uint16_t(readb(it,end)) << 8;
                    return ret;
                }

            template <class iterator> 
                void pushl(iterator & it, const iterator & end, uint32_t val) const {
                    pushw(it,end,val>>16);
                    pushw(it,end,val);
                }

            template <class iterator> 
                uint32_t readl(iterator & it, const iterator & end) const {
                    uint32_t ret = readw(it,end);
                    ret = uint32_t(readw(it,end)) | (ret << 16);
                    return ret;
                }

            template <class iterator> 
                bool checkb(iterator & it, const iterator & end, uint8_t vref, bool raise=false) const {
                    uint8_t val = readb(it,end);
                    if (raise) {
                        if (val != vref) {
                            throw std::runtime_error("Unexpected value in checkb");
                        }
                    }
                    return (val == vref);
                }

            template <class iterator> 
                bool checkw(iterator & it, const iterator & end, uint16_t vref, bool raise=false) const {
                    uint16_t val = readw(it,end);
                    if (raise) {
                        if (val != vref) {
                            throw std::runtime_error("Unexpected value in checkw");
                        }
                    }
                    return (val == vref);
                }

            template <class iterator> 
                bool checkl(iterator & it, const iterator & end, uint32_t vref, bool raise=false) const {
                    uint32_t val = readl(it,end);
                    if (raise) {
                        if (val != vref) {
                            throw std::runtime_error("Unexpected value in checkl");
                        }
                    }
                    return (val == vref);
                }


            
        public:
            Serializable() {}
            ~Serializable() {}

            uint32_t serialized_length() const {
                return 0;
            }

            template <class iterator>
                iterator deserialize(iterator it, const iterator & end) {
                    return it;
                }
            
            template <class iterator>
                iterator serialize(iterator it, const iterator & end) const {
                    return it;
                }

    };

};


#endif // LXS_LIB_SERIALIZATION_H
