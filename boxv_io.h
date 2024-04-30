/*****************************************************************************

Copyright (c) 2012-2022  Michal Necasek

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*****************************************************************************/

#include <miniport.h>
#include <ntddvdeo.h>
#include <video.h>
#include "videomp.h"

/* In this case, vid_in/out cannot be defined as macros because NT uses
 * a pretty funky interface for I/O port access. Actual functions are defined
 * in a separate module.
 */

extern void     vid_outb( void *cx, unsigned port, unsigned val );
extern void     vid_outw( void *cx, unsigned port, unsigned val );
extern unsigned vid_inb( void *cx, unsigned port );
extern unsigned vid_inw( void *cx, unsigned port );
extern unsigned vid_ind( void *cx, unsigned port );
